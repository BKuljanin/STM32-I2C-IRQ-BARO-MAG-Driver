/*
 * i2c.h
 *
 *  Created on: Jan 25, 2026
 *      Author: Bogdan Kuljanin
 */

#ifndef I2C_H_
#define I2C_H_

typedef enum {
    I2C_ST_IDLE = 0,

    // Start / addressing phases
    I2C_ST_START,          // SB is expected next
    I2C_ST_ADDR,         // ADDR is expected next after SLA+W/R
    I2C_ST_SEND_REG,       // send register address
    I2C_ST_RESTART,        // generate repeated START, SB expected

    // Data phases
    I2C_ST_TX,             // sending payload bytes (write)
    I2C_ST_RX,             // expecting receiving payload bytes (read)

    // Finish / error
    I2C_ST_STOP,           // generate stop / finalize
    I2C_ST_DONE,
    I2C_ST_ERROR
} i2c_state_t;

void I2C1_init(void);
i2c_status_t  I2C1_byteRead(char saddr, char maddr, char *data);
i2c_status_t  I2C1_burstWrite(char saddr, char maddr, int n, char* data);
i2c_status_t  I2C1_burstRead(char saddr, char maddr, int n, char* data);

#endif /* I2C_H_ */
