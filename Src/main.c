// now we want to receive the data on the microcontroller. We will shot that with the LED
// Run in debug mode and send data from putty/RealTerm
#include <stdio.h>
#include "stm32f4xx.h"
#include <stdint.h>
#include "uart.h"
#include "mpu6500.h"
#include "systick.h"

// Accelerometer data
int16_t acc_x, acc_y, acc_z;
float acc_xg,acc_yg,acc_zg;

// Gyroscope data
int16_t omega_x, omega_y, omega_z;
float omega_xg, omega_yg, omega_zg;
uint16_t gyro_calibration_samples = 500; // Gyroscope calibration number of samples

// Temperature data
int16_t temperature_raw;
float temperature_c;
extern uint8_t data_rec[14]; // Defined in mpu6500.c
extern int16_t gyro_bias[3]; // Defined in mpu6500.c

// Reading data flags
static uint8_t mpu_read_in_flight = 0;


int main(void)
{
	SysTick_Init();
	mpu6500_init();
	uint32_t next_read_ms = millis();   // schedule first read immediately

	// Gyroscope calibration
	mpu6500_calibrate_gyro(gyro_calibration_samples);

	while(1)
	{
			uint32_t now = millis(); // Reading current time

			// 10 ms task, reading MPU6500
			 if ((int32_t)(now - next_read_ms) >= 0)
			        {
				 	 	 	next_read_ms += 10U;

				 	 	 	if (!mpu_read_in_flight)
				 	 	 	{

							// Read from data register (accelerometer - temperature - gyroscope)
							// Reading from register where accelerometer data starts
							// Slave auto increments internal register address during sequential read (next data byte)
							if (I2C1_Read(DEVICE_ADDR, DATA_ACC_START_ADDR, 14, (char*)data_rec) == I2C_OK)
									{
							            mpu_read_in_flight = 1;
							        }
				 	 	 	}

			        }

						 if (mpu_read_in_flight && I2C1_Done())           // or driver function i2c_done()
							 {
							 mpu_read_in_flight = 0;

							 if (I2C1_Err() == I2C_OK)
							 {

							acc_x = ((data_rec[0]<<8) | data_rec[1]); // Shifting to obtain x, data is stored in 2 bytes
							acc_y = ((data_rec[2]<<8) | data_rec[3]); // Multiplied to get unit in g
							acc_z = ((data_rec[4]<<8) | data_rec[5]); // High byte first in this sensor, see the datasheet

							// Scaling values to [g] unit, as given in datasheet
							acc_xg = acc_x * 0.000122070;
							acc_yg = acc_y * 0.000122070;
							acc_zg = acc_z * 0.000122070;

							// Extracting temperature data
							temperature_raw = ((data_rec[6]<<8) | data_rec[7]);
							temperature_c =(temperature_raw / 333.87) + 21.0;

							omega_x = ((data_rec[8]<<8) | data_rec[9]); // Shifting to obtain omega_x, data is stored in 2 bytes
							omega_y = ((data_rec[10]<<8) | data_rec[11]);
							omega_z = ((data_rec[12]<<8) | data_rec[13]); // High byte first in this sensor, see the datasheet

							// Scaling values to deg/s unit, as given in datasheet
							omega_xg = (omega_x - gyro_bias[0]) / 65.6;
							omega_yg = (omega_y - gyro_bias[1]) / 65.6;
							omega_zg = (omega_z - gyro_bias[2]) / 65.6;

							 }
							 }




	}
}
