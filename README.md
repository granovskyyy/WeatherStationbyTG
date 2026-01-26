# WeatherStationbyTG

A simple weather station project based on the MKL05Z32 microcontroller.

This project implements a compact weather station using an ARM Cortex-M0+ MCU (NXP KL05Z) with temperature and humidity sensing, user interface, alarm thresholds, I2C communication, and LCD display.

---

##  Features

-  Temperature and humidity measurement using an SHT35 sensor via I2C  
-  LCD1602 display for real-time readout  
-  Configurable limits for Tmin, Tmax, Hmin, Hmax  
-  Visual alarm with RGB LEDs when values go outside configured thresholds  
-  Simple button-based user interface for adjusting limits  
-  Non-blocking scheduler using SysTick for responsive UI and tasks  
-  Debounced button handling and multi-button combinations for UX control

---

## Requirements

**Hardware:**

- NXP KL05Z / MKL05Z32 microcontroller (FRDM-KL05Z or custom PCB)  
- SHT35 I2C temperature/humidity sensor  
- LCD1602 + I2C backpack (optional, can be parallel driven)  
- RGB LEDs and push buttons (S1-S4)

**Toolchain / Software:**

- ARM GCC toolchain (tested on Keil uVision)
- Optional debugger (CMSIS-DAP / J-Link / etc.)

---

##  Pinout

| Device / Module    | Device Pin | MCU Pin (MKL05Z4) | Notes               |
| ------------------ | ---------- | ----------------- | ------------------- |
| **LCD 1602 (I2C)** | SDA        | PTB4 (SDA)        | I2C data line       |
|                    | SCL        | PTB3 (SCL)        | I2C clock line      |
|                    | VCC        | +5V               | LCD power supply    |
|                    | GND        | GND               | Ground              |
| **4x4 Keyboard**   | C1         | PTA7              | Column 1            |
|                    | C2         | PTA10             | Column 2            |
|                    | C3         | PTA11             | Column 3            |
|                    | C4         | PTA12             | Column 4            |
|                    | R1         | GND               | Common ground       |
| **RGB LED**        | Red        | PTB8              | Red channel         |
|                    | Green      | PTB9              | Green channel       |
|                    | Blue       | PTB10             | Blue channel        |
| **SHT35 Sensor**   | VCC        | +3.3V             | Sensor power supply |
|                    | GND        | GND               | Ground              |
|                    | SDA        | PTB4 (SDA)        | I2C data line       |
|                    | SCL        | PTB3 (SCL)        | I2C clock line      |

All I2C devices (LCD 1602 and SHT35 sensor) share the same I2C bus configured on pins PTB3 (SCL) and PTB4 (SDA).

---
## Block diagram 
<img width="1196" height="481" alt="image" src="https://github.com/user-attachments/assets/1b45af73-57f6-4515-a4ae-22ceb8d37568" />

##  Software Architecture

The main loop runs three tasks on timers (using a 1 ms SysTick tick):

| Task                | Interval | Purpose |
|---------------------|----------|---------|
| `UI_Task()`         | 20 ms    | Handles button input and UI state machine |
| `Measure_Task()`    | 200 ms   | Initiates and reads SHT35 sensor values |
| `LED_Task()`        | 100 ms   | Drives LED alarm logic with blink patterns |

This scheduler model avoids blocking loops (`for` or `delay`) ensuring responsive UI.

---

##  User Interface
- **S1 button:** exit configuration and return to main measurement screen
- **S2 button:** navigate through configurable fields (`Tmin`, `Tmax`, `Hmin`, `Hmax`)  
- **S3 button:** increment current field  
- **S4 button:** decrement current field  


Value limits are enforced in firmware:

- Temperature limits: `0°C ≤ Tmin < Tmax ≤ 40°C`  
- Humidity limits: `0% ≤ Hmin < Hmax ≤ 100%`

---

##  Alarm Behavior

- If temperature or humidity is outside range, respective LED (red/blue) lights steadily.  
- If *both* alarms are active simultaneously, LEDs alternate in a blink pattern **five times** and then settle.

---

##  Repository Structure
  WeatherStationbyTG/
  ├── main.c # Main application and task scheduler
  ├── lcd1602.c/.h # LCD driver for 1602 display
  ├── i2c.c/.h # I2C helper functions
  ├── Listings/Objects/RTE # Build configurations
  └── README.md

  ## Debugging (Keil uVision/JLink) 
  1. Project-> Options for Target
  2. In new window, choose Target -> ARM Compiler -> Use default compiler version 6
  3. In C/C++ window -> Language C -> c99
  4. In Debug window -> Use-> J-Link/J-TRACE Cortex
  5. Then Settings -> Accept -> Debug window -> Max Clock -> 2MHz 
  6. Flash Download window-> Reset and Run option 
  F7- project build
  F8- loading program on controller 

