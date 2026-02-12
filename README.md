# MPU6500 IMU (I2C Polling) Bare-Metal Driver on STM32 NUCLEO-F446RE

**Note:** **All peripherals (RCC, GPIO, SysTick, and I2C)** are configured manually using **bare-metal register access**.

This project reads the **MPU6500** IMU over **I2C1 (PB8/PB9)** and produces:
- Accelerometer data (X/Y/Z) in **g**
- Gyroscope data (X/Y/Z) in **deg/s**
- Temperature in **°C**
- Gyroscope bias calibration on startup to remove stationary offset

---

## Overview

The MPU6500 provides a continuous block of sensor output registers starting at **0x3B**:

- ACCEL_XOUT_H .. ACCEL_ZOUT_L (6 bytes)
- TEMP_OUT_H .. TEMP_OUT_L (2 bytes)
- GYRO_XOUT_H .. GYRO_ZOUT_L (6 bytes)

In the project **14 bytes** are read in a single burst read starting at **0x3B**.

**Key features:**
- **Bare-metal I2C** (STM32 I2C1 peripheral)
- **Polling-based** transfers (no DMA, no interrupts for I2C)
- **Timeout-protected wait loops** (prevents infinite stuck states)
- **NACK detection** via AF (Acknowledge Failure) flag
- **Abort + peripheral reset recovery** on timeout / NACK
- **SysTick 1ms timebase** used for delays and timeout measurement
- Main loop reads IMU every **10 ms** (100 Hz)

---

## Hardware Connections

| Signal | MPU6500 | STM32 NUCLEO-F446RE |
|-------:|---------|---------------------|
| VCC    | VCC     | 3.3V                |
| GND    | GND     | GND                 |
| SCL    | SCL     | PB8 (I2C1_SCL)      |
| SDA    | SDA     | PB9 (I2C1_SDA)      |
| AD0    | AD0     | GND                 |
| NCS    | NCS     | 3.3V                |

**Important:**
- I2C requires pull-up resistors on SDA/SCL (some modules have them onboard, some do not). MPU6500 has internal pull-up resistors.
- If the bus is unstable, verify pullups (typically 2.2k–10k depending on wiring length/capacitance). Also twist pairs of wires: SDA-GND and SCL-GND.

---

## Clock Assumptions

This project assumes:
- CPU clock = **16 MHz**
- SysTick uses processor clock (CTRL_CLKSRC = 1)

SysTick is configured to tick every 1 ms.
If SYSCLK changes, the SysTick timebase will change unless the reload value is updated.

---

## File Structure

### `main.c`
Program entry point. Initializes SysTick, MPU6500 sensor (by writing through I2C to the specific registers), performs gyro calibration once at startup, then reads the IMU data every 10 ms.

Main responsibilities:
- Scheduling reads using millisecond system clock
- Parsing the 14-byte sensor block into:
  - accelerometer raw + scaled
  - temperature raw + scaled
  - gyro raw + scaled (with bias subtraction)

---

### `systick.h / systick.c`
Provides a 1ms timebase using SysTick interrupt.

Functions:
- `void SysTick_Init(void);`
  - Configures SysTick for 1ms interrupts
- `uint32_t millis(void);`
  - Returns milliseconds since boot

SysTick ISR:
- `void SysTick_Handler(void);`
  - Increments an internal millisecond counter

---

### `i2c.h / i2c.c`
Bare-metal STM32F4 I2C1 driver with polling + timeouts.

Main ideas:
- Every blocking wait loop is replaced with a timeout-checked helper:
  - `wait_flag_set(...)`
  - `wait_flag_clear(...)`
  - `wait_flag_set_or_nack(...)`
- On timeout or NACK:
  - generate STOP
  - clear AF if set
  - disable/enable peripheral (CR1_PE)
  - verify BUSY clears (SR2_BUSY)

Functions:
- `void I2C1_init(void)`

  This function configures PB8 (SCL) and PB9 (SDA) as alternate function open-drain pins with pull-ups (required for I2C), enables the I2C1 peripheral clock, and performs a software reset of I2C peripheral. It     then configures the I2C timing registers (CR2, CCR, TRISE) for Standard Mode I2C (100 kHz) operation assuming a 16 MHz peripheral clock, and enables the I2C peripheral. After this function completes, I2C1 is     fully configured and ready for START.

- `i2c_status_t I2C1_byteRead(uint8_t saddr, uint8_t reg, char *data);`

  Single register, single byte read from an I2C slave using the following sequence:  
  Start - Slave address + W - (slave ACK) - Internal register sent as data bits - (slave ACK) - Repeated start - Slave address + R - (Slave sends data) - Master NACK - Stop
  
  During waiting for flags to be set up (START, ADDRESS, TXE, RXE) timer counts and if limit value is crossed, timeout is detected. Master also clears ADDR flag in read.
  
- `i2c_status_t I2C1_burstRead(uint8_t saddr, uint8_t start_reg, int n, char *buf);`

  Burst read from I2C slave. n bytes are read into the buffer. Sequence is similar to the function above. The difference is that after repeated start (reading has started), the master ACKs until the last byte.
  Only on the last remaining byte master NACs, indicating the end of the transfer. Stop then follows.

- `i2c_status_t I2C1_burstWrite(uint8_t saddr, uint8_t start_reg, int n, char *buf);`

  Burst write to I2C slave. n bytes are written into the slave. Start - Slave address + W - (slave ACK) - Register data byte - (slave ACK) - data - (slave ACK) -... - Stop.
  The slave auto-increments its internal register pointer during the burst, so consecutive bytes are written to consecutive registers.
  
- `static i2c_status_t i2c_abort_and_reset(i2c_status_t reason);`

  In case of timeout or error, I2C is aborted and the peripheral is reset.

  The rest are helper functions that measure time for timeout and trigger timeout actions and bus restoring.

  
Status codes:
- `I2C_OK`
- `I2C_ERR_TIMEOUT`
- `I2C_ERR_NACK`
- `I2C_ERR_BUSY`
- `I2C_ERR_RECOVERY`

---

### `mpu6500.h / mpu6500.c`
Initializes MPU6500 IMU sensor, reads it, and writes to it using I2C.

Functions:
- `void mpu6500_init(void);`
  
  Initializes the sensor by waking it up and configuring the measurement range for gyroscope and accelerometer.
  
- `void mpu6500_calibrate_gyro(uint16_t gyro_samples);`
  
  Calibrates the gyroscope by reading it for a given amount of times. Averages the measurements and calculates the offset from zero. The gyroscope must be still during this procedure.
  
- `void mpu6500_read_values(uint8_t reg);`

  Burst reads 14 bytes (6 for accelerometer ax, ay, az (H and L each), gyroscope (H and L each), and temperature H and L. 

---


## Reference Materials
All register settings, timer configurations, and GPIO modes in this code are implemented based on:

- **Reference Manual:** RM0390 Rev 7  
- **Datasheet:** DS10693 Rev 10  
- **User Manual:** UM1724 Rev 17
- **Cortex M4 Generic User Guide:** DUI 0553A

  
* **Sensor Documentation:** MPU-6500 Register Map and Descriptions Revision 2.1

