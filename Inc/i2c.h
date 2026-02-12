/*
 * i2c.h
 *
 *  Created on: Jan 25, 2026
 *      Author: Bogdan Kuljanin
 */

#ifndef I2C_H_
#define I2C_H_

typedef enum {
  I2C_OK = 0,
  I2C_ERR_TIMEOUT,
  I2C_ERR_NACK,
  I2C_ERR_BUSY,
  I2C_ERR_RECOVERY
} i2c_status_t;

void I2C1_init(void);
i2c_status_t  I2C1_byteRead(char saddr, char maddr, char *data);
i2c_status_t  I2C1_burstWrite(char saddr, char maddr, int n, char* data);
i2c_status_t  I2C1_burstRead(char saddr, char maddr, int n, char* data);

#endif /* I2C_H_ */
