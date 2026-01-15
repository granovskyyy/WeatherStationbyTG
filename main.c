#include "MKL05Z4.h"
#include "lcd1602.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

#define SHT35_ADDR  0x45
#define S1_MASK	(1<<9)		// Maska dla klawisza S1
#define S2_MASK	(1<<10)		// Maska dla klawisza S2
#define S3_MASK	(1<<11)		// Maska dla klawisza S3
#define S4_MASK	(1<<12)		// Maska dla klawisza S4
#define LED_R		(1<<8)		// Maska dla diody czerwonej (R)
#define LED_G		(1<<9)		// Maska dla diody czerwonej (G)
#define LED_B		(1<<10)		// Maska dla diody czerwonej (B)
#define S2_LIMIT 4
int T_min = 18, T_max = 24;
int H_min = 30, H_max = 60;
volatile uint8_t S2_press=0;	// "1" - klawisz zosta? wci?ni?ty "0" - klawisz "skonsumowany"
volatile uint8_t S3_press=0;
volatile uint8_t S4_press=0;
volatile uint8_t mode = 0;   // 0=Tmin,1=Tmax,2=Hmin,3=Hmax
volatile uint8_t ui_mode = 0;     // 0 = ekran glówny, 1 = konfiguracja
volatile uint32_t debounce=0;
volatile uint8_t screenmode=0;
volatile uint8_t S2_nr = 0;


void LED_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;      // W??czenie portu B
	PORTB->PCR[8] |= PORT_PCR_MUX(1);
	PORTB->PCR[9] |= PORT_PCR_MUX(1);	
	PORTB->PCR[10] |= PORT_PCR_MUX(1);
	PTB->PDDR |= (1<<8)|(1<<9)|(1<<10);	// Ustaw na 1 bity 8, 9 i 10 ? rola jako wyj?cia
	PTB->PDOR|= (1<<8)|(1<<9)|(1<<10);	// Zga? wszystkie diody
}

void Klaw_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;		// W??czenie portu A
	PORTA->PCR[9] |= PORT_PCR_MUX(1);
	PORTA->PCR[10] |= PORT_PCR_MUX(1);
	PORTA->PCR[11] |= PORT_PCR_MUX(1);
	PORTA->PCR[12] |= PORT_PCR_MUX(1);
	PORTA->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[10] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[12] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}
void Klaw_S2_4_Int(void)
{
	PORTA -> PCR[10] |= PORT_PCR_IRQC(0x8);		//0x8 - poziom "0"; 0x9 - zbocze narastajace; 0xa - zbocze opadajace; 0xb - obydwa zbocza
	PORTA -> PCR[11] |= PORT_PCR_IRQC(0x8);		
	PORTA -> PCR[12] |= PORT_PCR_IRQC(0x8);
	NVIC_SetPriority(PORTA_IRQn, 3); 
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler(void)	// Podprogram obs?ugi przerwania od klawiszy S2, S3 i S4
{
	uint32_t buf;
	buf=PORTA->ISFR & (S2_MASK | S3_MASK | S4_MASK);

	switch(buf)
	{
			case S2_MASK:    DELAY(100)
											if(!(PTA->PDIR & S2_MASK))
											{
													if(!S2_press)
													{
															S2_nr += 1;
															if(S2_nr > S2_LIMIT)
																	S2_nr = 0;
															S2_press = 1;
													}
											}
											break;
		case S3_MASK:	
			DELAY(100)		// Minimalizacja drga? zestyk?w
									if(!(PTA->PDIR&S3_MASK))		
									{
										if(!S3_press)
										{
											S3_press=1;
										}
									}
									break;
		case S4_MASK:	
			DELAY(100)
			if(!S4_press)
									{
										S4_press=1;
									}
									break;
		default:			break;
	}	
	PORTA->ISFR |=  S2_MASK | S3_MASK | S4_MASK;	// Kasowanie wszystkich bit?w ISF
	NVIC_ClearPendingIRQ(PORTA_IRQn);


}

int main(void)
{


    float temperature, humidity;
    uint8_t data[6];


    char buf[17];
		LED_Init();
		Klaw_Init();
		Klaw_S2_4_Int();
    I2C_Init();
	  LCD1602_Init();
		I2C_Ping(0x45);
		LCD1602_SetCursor(0,0);

		
    LCD1602_Backlight(TRUE);
    LCD1602_ClearAll();



    while(1)
    {
			

			if(!(PTA->PDIR & S1_MASK))   // S1 wcisniety
			{
					ui_mode = 0;             // powrót do podgladu
					screenmode = 1;         // odswiez LCD
			}
			if(debounce)
			{
				debounce--;
			}
			if(!debounce && S2_press)
			{
						if(S2_press)
						{
								ui_mode=1;
								mode = (mode + 1) % 4;
								screenmode=1;
								debounce=100000;
								S2_press = 0;
								continue;
						}
			}
						

				if(!debounce && ui_mode)
				{
						// ===== TRYB EDYCJI =====

	
						if(S3_press)
						{	
								ui_mode=1;
								switch(mode)
								{
									case 0:
										T_min++;
										break;
									case 1:
										T_max++;
										break;
									case 2:
										H_min++;
										break;
									case 3:
										H_max++;
										break;
									default:
										break;
								}
								screenmode=1;
								debounce=100000;
								S3_press = 0;
						}
					

						if(S4_press)
						{	
								ui_mode=1;
								switch(mode)
								{
									case 0:
										T_min--;
										break;
									case 1:
										T_max--;
										break;
									case 2:
										H_min--;
										break;
									case 3:
										H_max--;
										break;
									default:
										break;
								}
								screenmode=1;
								debounce=100000;
								S4_press = 0;
								


						}
					}
						
						if(ui_mode && screenmode)
						{
								screenmode = 0;

								LCD1602_ClearAll();

								switch(mode)
								{
										case 0:
												LCD1602_SetCursor(0,0);
												LCD1602_Print("Edit Tmin");
												LCD1602_SetCursor(0,1);
												sprintf(buf,"Val: %2d ",T_min);
												LCD1602_Print(buf);
												break;
										case 1:
												LCD1602_SetCursor(0,0);
												LCD1602_Print("Edit Tmax");
												LCD1602_SetCursor(0,1);
												sprintf(buf,"Val: %2d ",T_max);
												LCD1602_Print(buf);
												break;
										case 2:
												LCD1602_SetCursor(0,0);
												LCD1602_Print("Edit Hmin");
												LCD1602_SetCursor(0,1);
												sprintf(buf,"Val: %2d ",H_min);
												LCD1602_Print(buf);
												break;
										case 3:
												LCD1602_SetCursor(0,0);
												LCD1602_Print("Edit Hmax");
												LCD1602_SetCursor(0,1);
												sprintf(buf,"Val: %2d ",H_max);
												LCD1602_Print(buf);
												break;
								}
						}


					
			


        /* --- Start pomiaru SHT35 --- */
        I2C_WriteReg(SHT35_ADDR, 0x24, 0x00);   // wysyla 24 00 w jednej transakcji

        for(volatile int i=0;i<90000;i++);     // ~15 ms

        /* --- Odczyt 6 bajtów --- */
        I2C_ReadRegBlock(SHT35_ADDR, 0x00, 6, data);

        /* --- Przeliczenie --- */
        uint16_t rawT = ((uint16_t)data[0] << 8) | data[1];
        uint16_t rawH = ((uint16_t)data[3] << 8) | data[4];

        temperature = -45.0f + 175.0f * ((float)rawT / 65535.0f);
        humidity    = 100.0f * ((float)rawH / 65535.0f);
				
				
				uint8_t alarmT = (temperature < T_min || temperature > T_max);
				uint8_t alarmH = (humidity < H_min || humidity > H_max);

				PTB->PSOR = (1<<8)|(1<<9)|(1<<10);  // wylacz wszystkie

				if(alarmT) PTB->PCOR = LED_R;      // czerwona – temperatura
				if(alarmH) PTB->PCOR = LED_B;     // niebieska – wilgotnosc
				if(alarmT && alarmH)
				{
						for(int k=0; k<5; k++)
						{
								// temperatura - czerwony
								PTB->PCOR = LED_R;
								PTB->PSOR = LED_B;
								for(volatile int d=0; d<200000; d++);
								PTB->PCOR = LED_B;
								PTB->PSOR = LED_R;
								for(volatile int d=0; d<200000; d++);
						}
				}

        /* --- Formatowanie bez %f --- */
        int t_int = (int)temperature;
        int t_dec = (int)((temperature - t_int) * 10);
        int h_int = (int)humidity;
        int h_dec = (int)((humidity - h_int) * 10);

	
		
					


								LCD1602_SetCursor(0,0);
								sprintf(buf,"T: %2d.%1d C", t_int, t_dec);
								LCD1602_Print(buf);

								LCD1602_SetCursor(0,1);
								sprintf(buf,"H: %2d.%1d %%", h_int, h_dec);
								LCD1602_Print(buf);
						
				

			}


	}