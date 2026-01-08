#include "MKL05Z4.h"
#include "lcd1602.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

#define SHT35_ADDR  0x45

int main(void)
{
    float temperature, humidity;
    uint8_t data[6];
    char buf[17];

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

        /* --- Formatowanie bez %f --- */
        int t_int = (int)temperature;
        int t_dec = (int)((temperature - t_int) * 10);
        int h_int = (int)humidity;
        int h_dec = (int)((humidity - h_int) * 10);

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



