/*
 * mpu6500.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Bogdan Kuljanin
 */

#ifndef MPU6500_H_
#define MPU6500_H_

#include "i2c.h"

#include <stdint.h>

// The document MPU6500 will be refered
#define DEVID_R (0x75) // Device ID register WHO_AM_I (name)
#define DEVICE_ADDR (0x68) // p44 0x68 or 0x69 depending on AD0 pin. In this project AD0 is tied to GND so address is 0x68 according to the datasheet. Address of the slave
#define ACCEL_CONFIG_R (0x1C) // Data format p6 how sensitive should the sensor be (range)
#define GYRO_CONFIG_R (0x1B) // Data format gyroscope configuration
#define POWER_CTL_R (0x6B) // p8 register used to put the sensor into "Measurement Mode." Without writing to it, that sensor stays in standby.
#define DATA_ACC_START_ADDR (0x3B) // Start address of the accelerometer data register. Data is read starting from this address
#define DATA_GYRO_START_ADDR (0x43) // Start address of the gyroscope data register

#define FOUR_G (0x08) // +/-4 g measurement range
#define FIVE_HUNDRED_DPS (0x08) // +/- 500 dps p14 MPU6500 manual
#define RESET (0x00) // Wake up sensor
#define SET_MEASURE_B (0x00) // Wake up sensor, not required because the one above covers this



void mpu6500_init(void);
void mpu6500_read_values(uint8_t reg);
void mpu6500_calibrate_gyro(uint16_t gyro_samples);

#endif /* MPU6500_H_ */
