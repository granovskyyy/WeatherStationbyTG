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


//FUNKCJE SPRZETOWE
void Klaw_Init(void)
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

void LED_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;      // W??czenie portu B
	PORTB->PCR[8] |= PORT_PCR_MUX(1);
	PORTB->PCR[9] |= PORT_PCR_MUX(1);	
	PORTB->PCR[10] |= PORT_PCR_MUX(1);
	PTB->PDDR |= LED_R_MASK|LED_G_MASK|LED_B_MASK;	// Ustaw na 1 bity 8, 9 i 10 ? rola jako wyj?cia
	PTB->PDOR|= LED_R_MASK|LED_G_MASK|LED_B_MASK;	// Zga? wszystkie diody
}


/* ===== ZMIENNE GLOBALNE ===== */
volatile uint32_t sys_ms = 0;

int T_min = 18, T_max = 24;
int H_min = 30, H_max = 60;

volatile uint8_t S2_press = 0;
volatile uint8_t S3_press = 0;
volatile uint8_t S4_press = 0;
volatile uint8_t last_user_action=0;
uint8_t ui_mode = 0;
uint8_t mode = 0;
volatile uint8_t block_ui = 0;


float temperature, humidity;
uint8_t data[6];
char buf[17];

uint32_t blink_timer = 0;
uint8_t  blink_state = 0;
uint8_t  blink_count = 0;
uint8_t  blink_active = 0;

/* ===== PROTOTYPY ===== */
void LED_Init(void);
void Klaw_Init(void);
void Klaw_S2_4_Int(void);

void UI_Task(void);
void LCD_Task(void);
void Measure_Task(void);
void LED_Task(void);

/* ===== SYSTICK ===== */
void SysTick_Handler(void)
{
    sys_ms++;
}

/* ===== PRZERWANIE PORTA ===== */
void PORTA_IRQHandler(void)	// Podprogram obs?ugi przerwania od klawiszy S2, S3 i S4
{
	uint32_t buf;
	buf=PORTA->ISFR & (S2_MASK | S3_MASK | S4_MASK);

	switch(buf)
	{
		case S2_MASK:			// Minimalizacja drga? zestyk?w
									if(!(PTA->PDIR&S2_MASK))		
									{
										if(!S2_press)
										{
											S2_press=1;
										}
									}									
									break;
		case S3_MASK:			// Minimalizacja drga? zestyk?w
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
	PORTA->ISFR |=  S2_MASK | S3_MASK | S4_MASK;	// Kasowanie wszystkich bit?w ISF
	NVIC_ClearPendingIRQ(PORTA_IRQn);
}

/* ===== MAIN ===== */
int main(void)
{
    LED_Init();
    Klaw_Init();
    Klaw_S2_4_Int();
    I2C_Init();
    LCD1602_Init();

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);  // 1 ms tick

    uint32_t t_ui = 0, t_meas = 0, t_led = 0;

    LCD1602_Backlight(TRUE);
    LCD1602_ClearAll();

    while(1)
    {
        uint32_t now = sys_ms;

        if(now - t_ui >= 20)
        {
            t_ui = now;
            UI_Task();
        }

        if(now - t_meas >= 500)
        {
            t_meas = now;
            Measure_Task();
        }

        if(now - t_led >= 100)
        {
            t_led = now;
            LED_Task();
        }
    }
}

/* ===== UI TASK ===== */
void UI_Task(void)
{

    static uint32_t debounce = 0;

    if(debounce) debounce--;


    if(S2_press && S3_press)
		{
				ui_mode = 0;
				S2_press = 0;
				S3_press = 0;
				S4_press = 0;
		}


    //last_S1 = s1;

    if(!debounce && S2_press)
    {
				last_user_action = sys_ms;
        ui_mode = 1;
        mode = (mode + 1) % 4;
        debounce = 100;
        S2_press = 0;
    }

    if(!debounce && ui_mode && S3_press)
    {
				last_user_action = sys_ms;
        if(mode==0) T_min++;
        if(mode==1) T_max++;
        if(mode==2) H_min++;
        if(mode==3) H_max++;
        debounce = 100;
        S3_press = 0;
    }

    if(!debounce && ui_mode && S4_press)
    {
				last_user_action = sys_ms;
        if(mode==0) T_min--;
        if(mode==1) T_max--;
        if(mode==2) H_min--;
        if(mode==3) H_max--;
        debounce = 100;
        S4_press = 0;
    }

    LCD_Task();
}

/* ===== LCD TASK ===== */
void LCD_Task(void)
{
		static uint8_t last_ui_mode = 255;

		if(ui_mode != last_ui_mode)
		{
				LCD1602_ClearAll();
				last_ui_mode = ui_mode;
		}

    static uint8_t last_mode = 255;

    if(ui_mode)
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
    else
    {
        LCD1602_SetCursor(0,0);
        sprintf(buf,"T:%2d.%1dC",(int)temperature,(int)(temperature*10)%10);
        LCD1602_Print(buf);

        LCD1602_SetCursor(0,1);
        sprintf(buf,"H:%2d.%1d%%",(int)humidity,(int)(humidity*10)%10);
        LCD1602_Print(buf);
    }
}

/* ===== MEASURE TASK ===== */
void Measure_Task(void)
{
    I2C_WriteReg(SHT35_ADDR,0x24,0x00);
    for(volatile int i=0;i<90000;i++);

    I2C_ReadRegBlock(SHT35_ADDR,0x00,6,data);

    uint16_t rawT = (data[0]<<8)|data[1];
    uint16_t rawH = (data[3]<<8)|data[4];

    temperature = -45 + 175.0f * rawT / 65535.0f;
    humidity    = 100.0f * rawH / 65535.0f;
}

/* ===== LED TASK ===== */
void LED_Task(void)
{
    uint8_t alarmT = (temperature < T_min || temperature > T_max);
    uint8_t alarmH = (humidity < H_min || humidity > H_max);

    // Zgas wszystkie
    PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK;

    // === oba alarmy ===
    if(alarmT && alarmH)
    {
        if(!blink_active)   // start sekwencji
        {
            blink_active = 1;
            blink_count  = 0;
            blink_state  = 0;
            blink_timer  = sys_ms;
        }

        if(blink_active && (sys_ms - blink_timer >= 200))
        {
            blink_timer = sys_ms;
            blink_state ^= 1;

            if(!blink_state)
                blink_count++;   // zliczamy pelne migniecie
        }

        if(blink_state)
            PTB->PCOR = LED_R_MASK;
        else
            PTB->PCOR = LED_B_MASK;

        if(blink_count >= 5)
        {
            blink_active = 0;    // koniec migania
            PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK;
        }

        return;
    }

    // reset gdy alarm znika
    blink_active = 0;
    blink_count  = 0;

    // === pojedyncze alarmy ===
    if(alarmT) PTB->PCOR = LED_R_MASK;
    if(alarmH) PTB->PCOR = LED_B_MASK;
}
