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

float T_min = 18.0, T_max = 24.0;
float H_min = 30.0, H_max = 60.0;

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


int main(void)
{
    float temperature, humidity;
    uint8_t data[6];
    char buf[17];
		LED_Init();
		Klaw_Init();
    I2C_Init();
	  LCD1602_Init();
		I2C_Ping(0x45);
		LCD1602_SetCursor(0,0);
		uint8_t status;
		I2C_ReadReg(SHT35_ADDR, 0xF3, &status);  // status register
		LCD1602_SetCursor(0,1);
		if(status & 0x04)
				LCD1602_Print("Meas running");
		else
				LCD1602_Print("No meas     ");
		I2C_ReadRegBlock(0x44, 0x00, 6, data);
		sprintf(buf,"%02X %02X %02X", data[0], data[1], data[2]);
		LCD1602_SetCursor(0,0);
		LCD1602_Print(buf);

		
    LCD1602_Backlight(TRUE);
    LCD1602_ClearAll();



    while(1)
    {
				static uint8_t edit = 0;
				static uint8_t mode = 0; // 0:Tmin,1:Tmax,2:Hmin,3:Hmax

				if(!(PTA->PDIR & S1_MASK)) { mode = (mode + 1) % 4; for(volatile int i=0;i<30000;i++); }
				if(!(PTA->PDIR & S2_MASK)) { if(mode==0) T_min+=0.5; if(mode==1) T_max+=0.5; if(mode==2) H_min+=1; if(mode==3) H_max+=1; for(volatile int i=0;i<30000;i++); }
				if(!(PTA->PDIR & S3_MASK)) { if(mode==0) T_min-=0.5; if(mode==1) T_max-=0.5; if(mode==2) H_min-=1; if(mode==3) H_max-=1; for(volatile int i=0;i<30000;i++); }

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
				
				// --- czyszczenie linii LCD po alarmach ---
				LCD1602_SetCursor(0,0);
				LCD1602_Print("                ");
				LCD1602_SetCursor(0,1);
				LCD1602_Print("                ");

        /* --- LCD --- */
        sprintf(buf,"T: %2d.%1d C", t_int, t_dec);
        LCD1602_SetCursor(0,0);
        LCD1602_Print(buf);

        sprintf(buf,"H: %2d.%1d %%", h_int, h_dec);
        LCD1602_SetCursor(0,1);
        LCD1602_Print(buf);

        for(volatile int i=0;i<500000;i++);
    }
}