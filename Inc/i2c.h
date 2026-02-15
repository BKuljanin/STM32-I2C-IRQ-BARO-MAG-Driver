/*
 * i2c.h
 *
 *  Created on: Jan 25, 2026
 *      Author: Bogdan Kuljanin
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdio.h>
#include <stdint.h>

// Current operation Read/Write
typedef enum {
    I2C_OP_WRITE = 0,   // Operation==0  write
    I2C_OP_READ  = 1,   // Operation==1  read
} i2c_op_t;

typedef enum {
    I2C_ST_IDLE = 0,

    I2C_START,          // SB is expected next
    I2C_SADDR,         	// ADDR is expected next after SLA+W/R, slave address
    I2C_SEND_REG,       // send register address
	I2C_SEND_DATA,		// Sending data
	I2C_READ_DATA, 		// Reading data
    I2C_RESTART,        // generate repeated START, SB expected
	I2C_DATA_SENT, 		// Data written
	I2C_IDLE, 			// R/W done

} i2c_state_t;


typedef enum {
    I2C_OK = 0,
    I2C_BUSY,
    I2C_ERR_NACK,
    I2C_ERR_TIMEOUT,
    I2C_ERR_RECOVERY,
    I2C_ERR_OVERRUN,
    I2C_ERR_BUS,
    I2C_ERR_ARLO,
} i2c_status_t;


typedef struct {

    volatile i2c_state_t  st; // Current state
    volatile i2c_status_t err;     // set by ER IRQ or timeout code
    volatile uint8_t      done;    // 0 = busy, 1 = finished (OK or error)

    i2c_op_t              operation;     // 0 write, 1 read (as you use)
    uint8_t               saddr;         // 7-bit slave address
    uint8_t               maddr;         // Slave register address

    uint8_t              *data;          // Buffer pointer
    volatile uint16_t     n;             // Remaining bytes

    volatile uint8_t      addr_is_read;  // 0 = last addr sent was W, 1 = last addr sent was R
    volatile uint8_t      sent_reg;      // 0 = maddr not sent yet, 1 = maddr already sent

    volatile uint32_t     started_ms;    // for timeouts (set at start)
} i2c_struct;


void I2C1_init(void);
/*i2c_status_t  I2C1_byteRead(char saddr, char maddr, char *data);
i2c_status_t  I2C1_burstWrite(char saddr, char maddr, int n, char* data);
i2c_status_t  I2C1_burstRead(char saddr, char maddr, int n, char* data);*/

#endif /* I2C_H_ */
