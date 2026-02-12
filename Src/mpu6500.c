/*
 * mpu6500.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Bogdan Kuljanin
 */


#include "mpu6500.h"
#include "systick.h"


char data; // Global variable to store read values
uint8_t data_rec[14]; // 14 values to read:
// acceleration: x0,x1,y0,y1,z0,z1
// temperature: H and L
// angular velocity: x0,x1,y0,y1,z0,z1

int16_t gyro_bias[3];


void mpu6500_read_address (uint8_t reg)
{
	(void)I2C1_byteRead(DEVICE_ADDR, reg, &data);
}

void mpu6500_write (uint8_t reg, char value)
{
	char data[1];
	data[0]=value;

	(void)I2C1_burstWrite(DEVICE_ADDR, reg,1, data);
}

void mpu6500_read_values(uint8_t reg)
{
	(void)I2C1_burstRead(DEVICE_ADDR, reg, 14, (char *)data_rec); // 14 values to read

}

void mpu6500_init(void)
{
	// Enable I2C
	I2C1_init();

	// Read device ID, this should return 0x70 (WHO_AM_I)
	mpu6500_read_address(DEVID_R);

	// Set data format range for accelerometer +/- 4g
	mpu6500_write(ACCEL_CONFIG_R, FOUR_G);

	// Set data format range for gyroscope +/- 500 dps
	mpu6500_write(GYRO_CONFIG_R, FIVE_HUNDRED_DPS);

	// Wake up sensor
	mpu6500_write(POWER_CTL_R, RESET);

	// Configure power control measure bit
	mpu6500_write(POWER_CTL_R, SET_MEASURE_B); // For this sensor this line is the same as one above. One covers both power control and reset. Not required
}

void mpu6500_calibrate_gyro(uint16_t gyro_samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < gyro_samples; i++)
    {
        mpu6500_read_values(DATA_ACC_START_ADDR);

        sum_x += (int16_t)((data_rec[8]  << 8) | data_rec[9]);
        sum_y += (int16_t)((data_rec[10] << 8) | data_rec[11]);
        sum_z += (int16_t)((data_rec[12] << 8) | data_rec[13]);

        uint32_t t0 = millis();
        while ((millis() - t0) < 2) {}  // Delay between 2 readings in initialization. Always delay to not have consecutive reading commands
        // Give time between commanding read and previous read, bus internal state has to settle
    }

    gyro_bias[0] = sum_x / gyro_samples;
    gyro_bias[1] = sum_y / gyro_samples;
    gyro_bias[2] = sum_z / gyro_samples;
}
