#include "MKL05Z4.h"
#include "lcd1602.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

/* ===== DEFINICJE ===== */
#define SHT35_ADDR 0x45

#define S1_MASK (1<<9)
#define S2_MASK (1<<10)
#define S3_MASK (1<<11)
#define S4_MASK (1<<12)


#define LED_R_MASK		(1<<8)		// Maska dla diody czerwonej (R)
#define LED_G_MASK		(1<<9)		// Maska dla diody zielonej (G)
#define LED_B_MASK		(1<<10)		// Maska dla diody niebieskiej (B)

#define TMAXLIMIT 40
#define TMINLIMIT 0 
#define HMAXLIMIT 100
#define HMINLIMIT 0

//funkcje sprzetowe 
void Klaw_Init(void) //inicjalizacja klawiatury 
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Wlaczenie portu A
	PORTA->PCR[9] |= PORT_PCR_MUX(1);
	PORTA->PCR[10] |= PORT_PCR_MUX(1);
	PORTA->PCR[11] |= PORT_PCR_MUX(1);
	PORTA->PCR[12] |= PORT_PCR_MUX(1);
	PORTA->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[10] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[12] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}
void Klaw_S2_4_Int(void) 
{
	PORTA -> PCR[10] |= PORT_PCR_IRQC(0xa);		//0x8 - poziom "0"; 0x9 - zbocze narastajace; 0xa - zbocze opadajace; 0xb - obydwa zbocza
	PORTA -> PCR[11] |= PORT_PCR_IRQC(0xa);		
	PORTA -> PCR[12] |= PORT_PCR_IRQC(0xa);
	NVIC_SetPriority(PORTA_IRQn, 3); 
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void LED_Init(void) //obsluga diod LED
{
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;      // Wlaczenie portu B
	PORTB->PCR[8] |= PORT_PCR_MUX(1);
	PORTB->PCR[9] |= PORT_PCR_MUX(1);	
	PORTB->PCR[10] |= PORT_PCR_MUX(1);
	PTB->PDDR |= LED_R_MASK|LED_G_MASK|LED_B_MASK;	// Ustaw na 1 bity 8, 9 i 10 ? rola jako wyjscia
	PTB->PDOR|= LED_R_MASK|LED_G_MASK|LED_B_MASK;	// Zgas wszystkie diody
}


//zmienne globalne
volatile uint32_t sys_ms = 0;

int T_min = 18, T_max = 24; 
int H_min = 30, H_max = 60;

volatile uint8_t S2_press = 0; //flagi przyciskow s2-s4
volatile uint8_t S3_press = 0;
volatile uint8_t S4_press = 0;
volatile uint8_t ui_mode = 0;
volatile uint8_t mode = 0;
uint8_t meas_state = 0;      // 0 = idle, 1 = wait, 2 = read
uint32_t meas_timer = 0;

float temperature, humidity;
uint8_t data[6];
char buf[17];

uint32_t blink_timer = 0;
uint8_t  blink_state = 0;
uint8_t  blink_count = 0;
uint8_t  blink_active = 0;

//prototypy funkcji 
void LED_Init(void);
void Klaw_Init(void);
void Klaw_S2_4_Int(void);

void UI_Task(void);
void LCD_Task(void);
void Measure_Task(void);
void LED_Task(void);


void SysTick_Handler(void) //systick 
{
    sys_ms++;
}

void PORTA_IRQHandler(void)	// Podprogram obslugi przerwania od klawiszy S2, S3 i S4
{
	uint32_t buf;
	buf=PORTA->ISFR & (S2_MASK | S3_MASK | S4_MASK);

	switch(buf)
	{
		case S2_MASK:			// Minimalizacja drgan zestykow
									if(!(PTA->PDIR&S2_MASK))		
									{
										if(!S2_press)
										{
											S2_press=1;
										}
									}									
									break;
		case S3_MASK:			// Minimalizacja drgan zestykow
									if(!(PTA->PDIR&S3_MASK))		
									{
										if(!S3_press)
										{
											S3_press=1;
										}
									}
									break;
		case S4_MASK:	if(!S4_press)
									{
										S4_press=1;
									}
									break;
		default:			break;
	}	
	PORTA->ISFR |=  S2_MASK | S3_MASK | S4_MASK;	// Kasowanie wszystkich bitow ISF
	NVIC_ClearPendingIRQ(PORTA_IRQn);
}


int main(void) //main 
{
    LED_Init();
    Klaw_Init();
    Klaw_S2_4_Int();
    I2C_Init();
    LCD1602_Init();

    SystemCoreClockUpdate(); 
    SysTick_Config(SystemCoreClock / 1000);  //inicjacja systick (1 ms)

    uint32_t t_ui = 0, t_meas = 0, t_led = 0;

    LCD1602_Backlight(TRUE);
    LCD1602_ClearAll();

    while(1)
    {
        uint32_t now = sys_ms;

        if(now - t_ui >= 20) //odczyt przyciskow 
        {
            t_ui = now;
            UI_Task();
        }

        if(now - t_meas >= 500) //wykonywanie pomiarow 
        {
            t_meas = now;
            Measure_Task();
        }

        if(now - t_led >= 100) //miganie diodami LED 
        {
            t_led = now;
            LED_Task();
        }
    }
}

void UI_Task(void) //funkcja zarzadzajaca przyciskami do obslugi interfejsu uzytkownika 
{

    static uint32_t debounce = 0;
		static uint32_t tb =0;
    if(debounce) debounce--;


    if(S2_press && S3_press) // przytrzymanie s2 i s3 razem - powrót do menu pomiarowego 
		{
			if(tb==0)
			{
				tb=sys_ms;
			}
			if(sys_ms-tb>500) //czas przytrzymania po ktorym nastepuje wyjscie do menu glownego 
			{
				ui_mode = 0;
				S2_press = 0;
				S3_press = 0;
				S4_press = 0;
			}

		}
		else
		{
			tb=0;
		}

    if(!debounce && S2_press) //zmiana menu edycji za pomoca s2
    {
        ui_mode = 1;
        mode = (mode + 1) % 4; //zmiana trybu edycji (0- Tmin, 1- Tmax, 2- Hmin, 3- Hmax)
        debounce = 100;
        S2_press = 0;
    }

    if(!debounce && ui_mode && S3_press) //zwiekszanie parametru za pomoca s3 
    {
        if(mode==0)
				{
					if(T_min<TMAXLIMIT && T_min+1<T_max) //sprawdzanie czy Tmin jest mniejsze niz Tmax oraz czy nie przekracza limitu 
					{
						T_min++;
					}
				}					
        else if(mode==1) 
				{
					if(T_max<TMAXLIMIT) //sprawdzanie czy Tmax jest mniejsze niz limit Tmax 
					{
						T_max++;
					}
				}
        else if(mode==2) 
				{
					if(H_min<HMAXLIMIT && H_min+1<H_max) //sprawdzanie czy Hmin jest mniejsze niz Hmax oraz czy nie przekracza limitu
					{
						H_min++;
					}
				}			
        else if(mode==3)
				{
					if(H_max<HMAXLIMIT) //sprawdzanie czy Hmax jest mniejsze niz limit Hmax 
					{
						H_max++;
					}
				}					
        debounce = 100;
        S3_press = 0;
    }

    if(!debounce && ui_mode && S4_press) //s4 wcisniete - zmniejszanie sie parametru 
    {
        if(mode==0)
				{
					if(T_min > TMINLIMIT)
					{
						 T_min--;
					}
           
				}				
        else if(mode==1)
				{
					if(T_max>TMINLIMIT && T_max-1 >T_min)
					{
						T_max--;
					}
				}
        else if(mode==2)
				{
					if(H_min>HMINLIMIT)
					{
							H_min--;
					}
				}					
        else if(mode==3)
				{
					if(H_max>HMINLIMIT && H_max-1>H_min)
					{
								H_max--;
					}			
				}
        debounce = 100;
        S4_press = 0;
    }

    LCD_Task();  //wyswietlanie lcd 
}


void LCD_Task(void) //logika wyswietlania lcd 
{
		static uint8_t last_ui_mode = 255;

		if(ui_mode != last_ui_mode)
		{
				LCD1602_ClearAll();
				last_ui_mode = ui_mode;
		}

    static uint8_t last_mode = 255;

    if(ui_mode) //menu edycji 
    {
        if(mode != last_mode)
        {
            LCD1602_ClearAll();
            last_mode = mode;
        }

        LCD1602_SetCursor(0,0); 
        if(mode==0) LCD1602_Print("Edit Tmin");
        if(mode==1) LCD1602_Print("Edit Tmax");
        if(mode==2) LCD1602_Print("Edit Hmin");
        if(mode==3) LCD1602_Print("Edit Hmax");

        LCD1602_SetCursor(0,1);
        if(mode==0) sprintf(buf,"Val:%3dC",T_min);
        if(mode==1) sprintf(buf,"Val:%3dC",T_max);
        if(mode==2) sprintf(buf,"Val:%3d%%",H_min);
        if(mode==3) sprintf(buf,"Val:%3d%%",H_max);
        LCD1602_Print(buf);
    }
    else //menu pomiarów 
    {
        LCD1602_SetCursor(0,0);
        sprintf(buf,"T:%3d.%1dC",(int)temperature,(int)(temperature*10)%10);
        LCD1602_Print(buf);

        LCD1602_SetCursor(0,1);
        sprintf(buf,"H:%3d.%1d%%",(int)humidity,(int)(humidity*10)%10);
        LCD1602_Print(buf);
    }
}


void Measure_Task(void) //funkcja do wykonywania pomiarów 
{
    if(meas_state == 0)
    {
        I2C_WriteReg(SHT35_ADDR, 0x24, 0x00); //rozpoczecie pomiarów 
        meas_timer = sys_ms;
        meas_state = 1;
    }
    else if(meas_state == 1)
    {
        if(sys_ms - meas_timer >= 5) //odczyt po 5 milisekundach 
        {
            meas_state = 2;
        }
    }
    else if(meas_state == 2) //odczyt 
    {
        I2C_ReadRegBlock(SHT35_ADDR, 0x00, 6, data); //poczatek pomiaru 

        uint16_t rawT = (data[0] << 8) | data[1]; //odczyty z czujnika 
        uint16_t rawH = (data[3] << 8) | data[4];

        temperature = -45 + 175.0f * rawT / 65535.0f; //wyliczanie temperatury (wzor z datasheeta)
        humidity    = 100.0f * rawH / 65535.0f; //wyliczanie wilgotnosci 

        meas_state = 0;   // wróc do IDLE
				
				if(humidity < 0)   humidity = 0;  //limity wilgotnosci 
				if(humidity > 100) humidity = 100;

				if(temperature < -45) temperature = -45; //limity temperatur 
				if(temperature > 125) temperature = 125;

    }
}



void LED_Task(void) //obsluga diod led wraz z alarmami 
{
    uint8_t alarmT = (temperature < T_min || temperature > T_max); //alarm temperatury 
    uint8_t alarmH = (humidity < H_min || humidity > H_max); //alarm wilgotnosci 

    PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK; //gaszenie wszystkich diod 

    if(alarmT && alarmH) //sytuacja z dwoma alarmami 
    {
        if(!blink_active)   // start sekwencji
        {
            blink_active = 1;
            blink_count  = 0; //flaga do zliczania migniec 
            blink_state  = 0;
            blink_timer  = sys_ms; //systick mierzy czas migniec 
        }

        if(blink_active && (sys_ms - blink_timer >= 200))
        {
            blink_timer = sys_ms;
            blink_state ^= 1;

            if(!blink_state)
                blink_count++;   // zliczamy pelne migniecie
        }

        if(blink_state)
            PTB->PCOR = LED_R_MASK; //zaswiecamy czerwona diode 
        else
            PTB->PCOR = LED_B_MASK; //zaswiecami niebieska diode 

        if(blink_count >= 5) //5 migniec 
        {
            blink_active = 0;    
            PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK; //gasimy diody 
        }

        return;
    }

    blink_active = 0; //reset po alarmie 
    blink_count  = 0;


    if(alarmT) PTB->PCOR = LED_R_MASK; //czerwona dioda jak alarm temperatury 
    if(alarmH) PTB->PCOR = LED_B_MASK; //niebieska dioda jak alarm wilgotnosci 
}
