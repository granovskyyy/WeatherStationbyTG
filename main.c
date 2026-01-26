#include "MKL05Z4.h"
#include "lcd1602.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

/* ===== DEFINITIONS ===== */
#define SHT35_ADDR 0x45

#define S1_MASK (1<<7)     // Button masks
#define S2_MASK (1<<10)
#define S3_MASK (1<<11)
#define S4_MASK (1<<12)

#define LED_R_MASK (1<<8)  // Red LED mask
#define LED_G_MASK (1<<9)  // Green LED mask
#define LED_B_MASK (1<<10) // Blue LED mask

#define TMAXLIMIT 40       // Temperature limits
#define TMINLIMIT 0
#define HMAXLIMIT 100      // Humidity limits
#define HMINLIMIT 0

/* ===== GLOBAL VARIABLES ===== */
volatile uint32_t sys_ms = 0;        // SysTick time counter (ms)
uint32_t boot_start_ms = 0;

int T_min = 18, T_max = 24;          // Temperature limits
int H_min = 30, H_max = 60;          // Humidity limits

volatile uint8_t S2_press = 0;       // Button press flags (S2–S4)
volatile uint8_t S3_press = 0;
volatile uint8_t S4_press = 0;

volatile uint8_t ui_mode = 0;        // UI mode: 0 = measurement, 1 = edit
volatile uint8_t edit_mode = 0;      // Edited parameter:
                                     // 0=Tmin, 1=Tmax, 2=Hmin, 3=Hmax

uint8_t meas_state = 0;              // Measurement state:
                                     // 0 = idle, 1 = wait, 2 = read
uint32_t meas_timer = 0;             // Measurement timer

float temperature, humidity;         // Measured temperature and humidity
uint8_t data[6];                     // Raw data from SHT35 (6 bytes)
char buf[17];                        // LCD text buffer

uint32_t blink_timer = 0;            // Alarm blinking timer
uint8_t  blink_state = 0;            // Current blink state
uint8_t  blink_count = 0;            // Blink counter
uint8_t  blink_active = 0;           // Alarm active flag

/* ===== HARDWARE FUNCTIONS ===== */
void Klaw_Init(void)                 // Keyboard initialization
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  // Enable PORTA clock

    PORTA->PCR[7]  |= PORT_PCR_MUX(1);
    PORTA->PCR[10] |= PORT_PCR_MUX(1);
    PORTA->PCR[11] |= PORT_PCR_MUX(1);
    PORTA->PCR[12] |= PORT_PCR_MUX(1);

    PORTA->PCR[7]  |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[10] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[12] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}

void Klaw_S2_4_Int(void)              // Interrupt initialization for buttons
{
    PORTA->PCR[10] |= PORT_PCR_IRQC(0xA); // Falling edge interrupt
    PORTA->PCR[11] |= PORT_PCR_IRQC(0xA);
    PORTA->PCR[12] |= PORT_PCR_IRQC(0xA);

    NVIC_SetPriority(PORTA_IRQn, 3);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

void LED_Init(void)                  // LED initialization
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable PORTB clock

    PORTB->PCR[8]  |= PORT_PCR_MUX(1);
    PORTB->PCR[9]  |= PORT_PCR_MUX(1);
    PORTB->PCR[10] |= PORT_PCR_MUX(1);

    PTB->PDDR |= LED_R_MASK | LED_G_MASK | LED_B_MASK; // Set LEDs as outputs
    PTB->PDOR |= LED_R_MASK | LED_G_MASK | LED_B_MASK; // Turn off all LEDs
}

/* ===== SYSTICK HANDLER ===== */
void SysTick_Handler(void)
{
    sys_ms++;                         // Increment system time (ms)
}

/* ===== PORTA INTERRUPT HANDLER ===== */
void PORTA_IRQHandler(void)
{
    uint32_t buf;
    buf = PORTA->ISFR & (S2_MASK | S3_MASK | S4_MASK);

    switch(buf)
    {
        case S2_MASK:                // Button S2 pressed
            if(!(PTA->PDIR & S2_MASK))
                S2_press = 1;
            break;

        case S3_MASK:                // Button S3 pressed
            if(!(PTA->PDIR & S3_MASK))
                S3_press = 1;
            break;

        case S4_MASK:                // Button S4 pressed
            S4_press = 1;
            break;

        default:
            break;
    }

    PORTA->ISFR |= S2_MASK | S3_MASK | S4_MASK; // Clear interrupt flags
    NVIC_ClearPendingIRQ(PORTA_IRQn);
}

/* ===== LCD TASK ===== */
void LCD_Task(void)                  // LCD display logic
{
    static uint8_t last_ui_mode = 255;
    static uint8_t last_mode = 255;

    if(ui_mode != last_ui_mode)
    {
        LCD1602_ClearAll();
        last_ui_mode = ui_mode;
    }

    if(ui_mode)                      // Edit menu
    {
        if(edit_mode != last_mode)
        {
            LCD1602_ClearAll();
            last_mode = edit_mode;
        }

        LCD1602_SetCursor(0,0);
        if(edit_mode==0) LCD1602_Print("Edit Tmin");
        if(edit_mode==1) LCD1602_Print("Edit Tmax");
        if(edit_mode==2) LCD1602_Print("Edit Hmin");
        if(edit_mode==3) LCD1602_Print("Edit Hmax");

        LCD1602_SetCursor(0,1);
        if(edit_mode==0) sprintf(buf,"Val:%3dC",T_min);
        if(edit_mode==1) sprintf(buf,"Val:%3dC",T_max);
        if(edit_mode==2) sprintf(buf,"Val:%3d%%",H_min);
        if(edit_mode==3) sprintf(buf,"Val:%3d%%",H_max);
        LCD1602_Print(buf);
    }
    else                             // Measurement screen
    {
        LCD1602_SetCursor(0,0);
        sprintf(buf,"T:%3d.%1dC",(int)temperature,(int)(temperature*10)%10);
        LCD1602_Print(buf);

        LCD1602_SetCursor(0,1);
        sprintf(buf,"H:%3d.%1d%%",(int)humidity,(int)(humidity*10)%10);
        LCD1602_Print(buf);
    }
}

/* ===== UI TASK ===== */
void UI_Task(void)                   // User interface control
{
    static uint32_t debounce = 0;
    if(debounce) debounce--;

    if(!(PTA->PDIR & S1_MASK))        // S1 pressed – return to measurement mode
    {
        ui_mode = 0;
        S2_press = S3_press = S4_press = 0;
        debounce = 50;
    }

    if(!debounce && S2_press)         // Change edited parameter
    {
        ui_mode = 1;
        edit_mode = (edit_mode + 1) % 4;
        debounce = 50;
        S2_press = 0;
    }

    if(!debounce && ui_mode && S3_press) // Increase parameter value
    {
        if(edit_mode==0 && T_min<TMAXLIMIT && T_min+1<T_max) T_min++;
        else if(edit_mode==1 && T_max<TMAXLIMIT) T_max++;
        else if(edit_mode==2 && H_min<HMAXLIMIT && H_min+1<H_max) H_min++;
        else if(edit_mode==3 && H_max<HMAXLIMIT) H_max++;

        debounce = 50;
        S3_press = 0;
    }

    if(!debounce && ui_mode && S4_press) // Decrease parameter value
    {
        if(edit_mode==0 && T_min>TMINLIMIT) T_min--;
        else if(edit_mode==1 && T_max>TMINLIMIT && T_max-1>T_min) T_max--;
        else if(edit_mode==2 && H_min>HMINLIMIT) H_min--;
        else if(edit_mode==3 && H_max>HMINLIMIT && H_max-1>H_min) H_max--;

        debounce = 50;
        S4_press = 0;
    }

    LCD_Task();
}

/* ===== MEASUREMENT TASK ===== */
void Measure_Task(void)               // Temperature & humidity measurement
{
    if(meas_state == 0)
    {
        I2C_WriteReg(SHT35_ADDR, 0x24, 0x00); // Start measurement
        meas_timer = sys_ms;
        meas_state = 1;
    }
    else if(meas_state == 1)
    {
        if(sys_ms - meas_timer >= 5)  // Wait ~5 ms
            meas_state = 2;
    }
    else if(meas_state == 2)
    {
        I2C_ReadRegBlock(SHT35_ADDR, 0x00, 6, data);

        uint16_t rawT = (data[0] << 8) | data[1]; // Raw temperature value (MSB+LSB)
        uint16_t rawH = (data[3] << 8) | data[4]; // Raw humidity value (MSB+LSB)

        temperature = -45 + 175.0f * rawT / 65535.0f;
        humidity    = 100.0f * rawH / 65535.0f;

        meas_state = 0;

        if(humidity < 0) humidity = 0;
        if(humidity > 100) humidity = 100;
        if(temperature < -45) temperature = -45;
        if(temperature > 125) temperature = 125;
    }
}

/* ===== LED TASK ===== */
void LED_Task(void)                   // LED alarm handling
{
    uint8_t alarmT = (temperature < T_min || temperature > T_max);
    uint8_t alarmH = (humidity < H_min || humidity > H_max);

    PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK;

    if(alarmT && alarmH)
    {
        if(!blink_active)
        {
            blink_active = 1;
            blink_count  = 0;
            blink_state  = 0;
            blink_timer  = sys_ms;
        }

        if(sys_ms - blink_timer >= 200)
        {
            blink_timer = sys_ms;
            blink_state ^= 1;
            if(!blink_state) blink_count++;
        }

        if(blink_state) PTB->PCOR = LED_R_MASK;
        else            PTB->PCOR = LED_B_MASK;

        if(blink_count >= 5)
        {
            blink_active = 0;
            PTB->PSOR = LED_R_MASK | LED_G_MASK | LED_B_MASK;
        }
        return;
    }

    blink_active = 0;
    blink_count  = 0;

    if(alarmT) PTB->PCOR = LED_R_MASK;
    if(alarmH) PTB->PCOR = LED_B_MASK;
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
    SysTick_Config(SystemCoreClock / 1000); // 1 ms SysTick

    uint32_t t_ui = 0, t_meas = 0, t_led = 0;

    LCD1602_Backlight(TRUE);
    LCD1602_ClearAll();

    LCD1602_SetCursor(0,0);            // Welcome screen
    LCD1602_Print("Weather Station");
    LCD1602_SetCursor(0,1);
    LCD1602_Print("Starting...");

    boot_start_ms = sys_ms;
    while(sys_ms - boot_start_ms < 1500);

    while(1)
    {
        if(sys_ms - t_ui >= 20)
        {
            t_ui = sys_ms;
            UI_Task();
        }

        if(sys_ms - t_meas >= 200)
        {
            t_meas = sys_ms;
            Measure_Task();
        }

        if(sys_ms - t_led >= 100)
        {
            t_led = sys_ms;
            LED_Task();
        }
    }
}

