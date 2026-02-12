#include "stm32f4xx.h"
#include "systick.h"
#include "i2c.h"

#define GPIOBEN (1U<<1)
#define I2C1EN (1U<<21)
#define CR1_PE (1U<<0)

//read function
#define SR2_BUSY (1U<<1)
#define CR1_START (1U<<8)
#define SR1_SB (1U<<0)
#define SR1_ADDR (1U<<1)
#define SR1_TXE (1U<<7)
#define CR1_ACK (1U<<10)
#define CR1_STOP (1U<<9)
#define SR1_RXNE (1U<<6)
#define SR1_BTF (1U<<2)

#define I2C_100KHZ 80 //0B 0101 000 decimal = 80
#define SD_MODE_MAX_RISE_TIME 17 // various tools compute this value

// Timeout constants
#define I2C_FLAG_TIMEOUT_MS   5U
#define I2C_BUSY_TIMEOUT_MS   5U

// Helper functions
#define I2C_SR1_AF (1U<<10)
#define I2C_SR2_BUSY (1U<<1)

// Prototypes of helper functions
static i2c_status_t wait_flag_set_checked(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms);
static i2c_status_t wait_flag_clear_checked(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms);
static i2c_status_t wait_flag_set_or_nack_checked(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms);
static i2c_status_t i2c_abort_and_reset(i2c_status_t reason);


void I2C1_init(void)
{

	// Enable clock access for GPIOB
	RCC->AHB1ENR |= GPIOBEN;

	// Configure SCL and SDA lines of I2C1
	// Datasheet p58 alternate function PB8 SCL, PB9 SDA
	GPIOB->MODER &=~(1U<<16);
	GPIOB->MODER |=(1U<<17);
	GPIOB->MODER &=~(1U<<18);
	GPIOB->MODER |=(1U<<19); // in mode register

	// Set output type PB8 and PB9 to open drain (required for i2c), ref manual p186, write 1 for open drain
	GPIOB->OTYPER |= (1U<<8); // OT 8 and OT9 for PB8 and PB9
	GPIOB->OTYPER |= (1U<<9);

	// Enable Pullup for PB8,PB9
	GPIOB->PUPDR |= (1U<<16); // PB8 bits 16,17
	GPIOB->PUPDR &=~ (1U<<17); // 01 pullup

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &=~ (1U<<19);

	// set alternate function type
	GPIOB->AFR[1] &=~ (1U<<0);
	GPIOB->AFR[1] &=~ (1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~ (1U<<3);

	GPIOB->AFR[1] &=~ (1U<<4);
	GPIOB->AFR[1] &=~ (1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~ (1U<<7);

	RCC->APB1ENR |= I2C1EN; // Enable clock access for I2C1, Reference manual p146 bit 21

	// Enter reset mode
	I2C1->CR1 |= (1U<<15); // Reference manual p763, when HIGH, I2C is in reset state

	// Exit reset mode
	I2C1->CR1 &= ~(1U<<15);

	/*Configure peripheral clock*/
	I2C1->CR2 = (1U<<4);// I2C bus frequency. Reference manual p765. 16MHz (10000 binary 5th bit position 5)
	//If clock is 1MHz simply write 1 since its all MHz, set bit 0 to 1; if 2Mhz write to bit 1 (boolean 2)

	I2C1->CCR = I2C_100KHZ; //CCR bits 0-11 I2C clock. We want standard mode 100kHz, 16MHz/(2*80) = 100kHz

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;// Rise time, reference manual p772,773. The time it takes for the I²C signal (SDA or SCL) to go from LOW to HIGH

	//Enable I2C
	I2C1->CR1 |= CR1_PE;
}

// Read byte funciton
// 3 arguments, 1st address of slave, 2nd , 3rd pointer where to store
i2c_status_t I2C1_byteRead(char saddr, char maddr, char* data){

	volatile int tmp;

	/* First part of the message, writing to slave to set up internal register pointer
	 * start bit - slave address+W - slave memory address*/

	// Checking I2C busy flag
	if (wait_flag_clear_checked(&I2C1->SR2, SR2_BUSY, I2C_BUSY_TIMEOUT_MS) != I2C_OK) // Make sure the bus is not busy, check busy flag
	    return I2C_ERR_TIMEOUT; // These are all simple while(!())... functions but timeout and recover are built on, look at the bottom

	// Generating start condition
	I2C1->CR1 |= CR1_START; // Once its not busy, generate start condition, set start bit in CR1. Bit 8 start bit Reference manual p763

	// Wait until start bit set
	if (wait_flag_set_checked(&I2C1->SR1, SR1_SB, I2C_FLAG_TIMEOUT_MS) != I2C_OK) // Reference manual p767-770 SB start bit 0; 1: start condition generated
	    return I2C_ERR_TIMEOUT;

	// Transmit slave address + write
	I2C1->DR = saddr <<1; // Reference manual p767. This moves it into bits [7:1] and clears bit 0. R/W 0 = WRITE. So we type slave address, shift one left and that way bit 0 is 0 which is write

	// Wait until address flag is set
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_ADDR, I2C_FLAG_TIMEOUT_MS) != I2C_OK)  // Wait until the slave ACKs the address
	    return I2C_ERR_TIMEOUT;

	// Clear address flag, clear it after the loop
	tmp = I2C1->SR1;
	tmp = I2C1->SR2; // Clear by reading SR First SR1 then SR2 according to reference manual p770

	// Now we can send memory address, send to data register
	I2C1->DR = maddr;

	// Wait until data register empty, check TXE flag
	//while(!(I2C1->SR1 & SR1_TXE)){} // Reference manual p769 bit 7.
	// Wait until the transmit data register is empty, indicating the previous byte has been loaded into the shift register and the next byte can be written
	// while(!(I2C1->SR1 & SR1_BTF)) {} Possibly include this line instead, which is byte transfer finished (unlike TXE which is only DR empty, but byte might be on bus)
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_TXE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
	    return I2C_ERR_TIMEOUT;

	/* Second part of the message, reading from slave starts
	 * repeated start bit - slave address+R - reading data from slave*/

	// Generate another start condition - restart condition, same line as above
	I2C1->CR1 |= CR1_START; // Reference manual p764

	// Wait until start bit set
	if (wait_flag_set_checked(&I2C1->SR1, SR1_SB, I2C_FLAG_TIMEOUT_MS) != I2C_OK) // Reference manual p767-770 SB start bit 0; 1: start condition generated
		    return I2C_ERR_TIMEOUT;

	// Transmit slave address + read
	I2C1->DR = saddr <<1 | 1; // Shift slave address and sets bit 0 which is read

	// Wait until address flag is set
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_ADDR, I2C_FLAG_TIMEOUT_MS) != I2C_OK)  // Wait until the slave ACKs the address
		    return I2C_ERR_TIMEOUT;

	// Disable acknowledge
	I2C1->CR1 &=~ CR1_ACK; // In the read, the master controls ACK, and thats why ACK is cleared here (on top it wasnt cleared since in write slave ACKs)
	// Preconfigures the MCU (master) to send a NACK after the next received data byte, indicating that no further bytes are requested.
	// This is required for a single byte read, because in I2C READ, the master uses NACK to tell the slave to stop transmitting, and STOP only terminates the bus afterward.
	// On the other side, ACK means keep sending data

	// Clear address flag by reading SR
	tmp = I2C1->SR1;
	tmp = I2C1->SR2;

	// Generate stop after data received
	I2C1->CR1 |= CR1_STOP; // Reference manual p764

	// Data register not empty
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_RXNE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
	    return I2C_ERR_TIMEOUT; // Wait until one data byte has been received from the slave and is ready to be read

	*data++ = I2C1->DR; // Store received I2C byte in buffer and increment pointer. Will be useful for burst read

	return I2C_OK;
}


i2c_status_t I2C1_burstRead(char saddr, char maddr, int n, char* data)
{
	// n number of bytes to read
	// See the function above for detailed comments

	volatile int tmp;

	/* First part of the message, writing to slave to set up internal register pointer
	 * start bit - slave address+W - slave memory address*/

	// Checking I2C busy flag
	if (wait_flag_clear_checked(&I2C1->SR2, SR2_BUSY, I2C_BUSY_TIMEOUT_MS) != I2C_OK) // Make sure the bus is not busy, check busy flag
	    return I2C_ERR_TIMEOUT;

	// Generating start condition
	I2C1->CR1 |= CR1_START; // once its not busy, generate start condition, set start bit in cr1. bit 8 start bit p763

	// Wait until start bit set
	if (wait_flag_set_checked(&I2C1->SR1, SR1_SB, I2C_FLAG_TIMEOUT_MS) != I2C_OK) // Reference manual p767-770 SB start bit 0; 1: start condition generated
		    return I2C_ERR_TIMEOUT;

	// transmit slave address + write
	I2C1->DR = saddr <<1; // ref man p767. This moves it into bits [7:1] and clears bit 0. R/W 0 = WRITE. So we type slave address, shift one left and that way bit 0 is 0 which is write

	// Transmit slave address + write
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_ADDR, I2C_FLAG_TIMEOUT_MS) != I2C_OK)  // Wait until the slave ACKs the address
	    return I2C_ERR_TIMEOUT;

	// Clear address flag
	tmp = I2C1->SR1;
	tmp = I2C1->SR2;

	// Wait until data register empty, check TXE flag
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_TXE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
	    return I2C_ERR_TIMEOUT;

	// Now we can send memory address, send to data register
	I2C1->DR = maddr;

	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_TXE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
	    return I2C_ERR_TIMEOUT;

	/* Second part of the message, reading from slave starts
	 * repeated start bit - slave address+R - reading data from slave*/

	// Generate another start condition - restart condition, same line as above
	I2C1->CR1 |= CR1_START;

	// Wait until start bit set
	if (wait_flag_set_checked(&I2C1->SR1, SR1_SB, I2C_FLAG_TIMEOUT_MS) != I2C_OK) // Reference manual p767-770 SB start bit 0; 1: start condition generated
			    return I2C_ERR_TIMEOUT;

	// Transmit slave address + read
	I2C1->DR = saddr <<1 | 1;

	// Wait until address flag is set
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_ADDR, I2C_FLAG_TIMEOUT_MS) != I2C_OK)  // Wait until the slave ACKs the address
	    return I2C_ERR_TIMEOUT;

	// Clear address flag, clear it after the loop. Clear by reading SR
	tmp = I2C1->SR1;
	tmp = I2C1->SR2;

	// Enable acknowledge
	I2C1->CR1 |= CR1_ACK; // Enable ACK so the master acknowledges each received byte, indicating that more data is requested from the slave (unlike NACK in previous function where we wanted 1 byte)

	while(n>0U)
	{
		// If one byte remains
		if(n==1U){
			// Disable acknowledge, on next byte received send NACK indicating end of burst read (no more data to be read)
			I2C1->CR1 &=~ CR1_ACK;

			// Generate stop condition to terminate the I2C transaction after the last byte
			I2C1->CR1 |= CR1_STOP;

			// Data register not empty
			if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_RXNE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
			    return I2C_ERR_TIMEOUT; // Wait until data byte received from slave

			// Store received I2C byte in buffer and increment pointer
			*data++ = I2C1->DR; // data is the pointer to the vector buffer
			// 1) Store the byte into the location pointed to by data
			// 2) Increment the pointer data to point to the next buffer position

			/* Write the received byte to the current buffer element
			buf[0], then buf[1], then buf[2], ...
			Increment the pointer
			data now points to the next element in data vector */

			break;
		}
		else // More than 1 byte remaining to read
		{
			// Data register not empty
			if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_RXNE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
			    return I2C_ERR_TIMEOUT;

			// Read data from DR
			(*data++) = I2C1->DR; // See above

			// Update the number of bytes left to read
			n--;
		}

	}
	return I2C_OK;
}

i2c_status_t I2C1_burstWrite(char saddr, char maddr, int n, char* data){
// Slave address, memory address, number of bytes to write, pointer data
	volatile int tmp;

	// Wait until bus not busy
	if (wait_flag_clear_checked(&I2C1->SR2, SR2_BUSY, I2C_BUSY_TIMEOUT_MS) != I2C_OK) // Make sure the bus is not busy, check busy flag
	    return I2C_ERR_TIMEOUT;

	// Generate start condition
	I2C1->CR1 |= CR1_START;

	// Waiting start condition
	if (wait_flag_set_checked(&I2C1->SR1, SR1_SB, I2C_FLAG_TIMEOUT_MS) != I2C_OK) // Reference manual p767-770 SB start bit 0; 1: start condition generated
		    return I2C_ERR_TIMEOUT;

	// Transmit slave address + write
	I2C1->DR = saddr <<1;

	// Wait until address flag is set
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_ADDR, I2C_FLAG_TIMEOUT_MS) != I2C_OK)  // Wait until the slave ACKs the address
		    return I2C_ERR_TIMEOUT;

	// Clear address flag
	tmp = I2C1->SR1;
	tmp = I2C1->SR2;

	// Wait until data register empty, check TXE flag
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_TXE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
		    return I2C_ERR_TIMEOUT;

	// Now we can send memory address to data register
	I2C1->DR = maddr;

	// Now we can write in for loop
	for (int i =0;i<n;i++){

		// Wait until data register is ready
		if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_TXE, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
			    return I2C_ERR_TIMEOUT; // Wait until the transmit data register is empty so the next byte can be written

		// Writing data to DR
		I2C1->DR = *data++; // Write the current buffer byte to DR and move the data pointer to the next element

	}

	// Wait until the byte transfer is finished
	if (wait_flag_set_or_nack_checked(&I2C1->SR1, SR1_BTF, I2C_FLAG_TIMEOUT_MS) != I2C_OK)
	    return I2C_ERR_TIMEOUT;
 // BTF set, indicates the data byte has been transmitted and the shift register and DR are both empty
	// Reference manual p770 but 2 byte transfer finished, waiting. 1: data byte transfer succeeded

	// Generate stop condition
	I2C1->CR1 |= CR1_STOP; // Generate stop after data received to terminate the I2C transaction after the last byte

	return I2C_OK;
}



/* Handling timeouts, re-enabling bus, implementing helper function */

// Elapsed time helper function
static inline uint32_t elapsed_ms(uint32_t start)
{
    return (uint32_t)(millis() - start);
}

// Wait for flag set
static i2c_status_t wait_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms)
{
    uint32_t start = millis(); // Obtain current time

    while (((*reg) & mask) == 0U) { // Counts while register value equals mask value. Stops when reg = mask
        if (elapsed_ms(start) >= timeout_ms) // Checking for timeout
            return I2C_ERR_TIMEOUT;
    }
    return I2C_OK;
}

// Wait for flag clear
static i2c_status_t wait_flag_clear(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms)
{
    uint32_t start = millis(); // Obtain current time

    while (((*reg) & mask) != 0U) { // Counts while register value doesnt equal mask value. Stops when reg != mask
        if (elapsed_ms(start) >= timeout_ms) // Checking for timeout
            return I2C_ERR_TIMEOUT;
    }
    return I2C_OK;
}

// Wait for flag or detect NACK
static i2c_status_t wait_flag_set_or_nack(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms)
{
    uint32_t start = millis();

    while (((*reg) & mask) == 0U) {

        if (I2C1->SR1 & I2C_SR1_AF) {   // NACK. Acknowledge failure, reference manual p768
            I2C1->SR1 &= ~I2C_SR1_AF;  // Clear AF, unblocks the peripheral. So clearing AF is mandatory to: allow STOP to complete, allow a new START, prevent permanent BUSY state
            return I2C_ERR_NACK;
        }

        if (elapsed_ms(start) >= timeout_ms)
            return I2C_ERR_TIMEOUT;
    }
    return I2C_OK;
}

// Abort plus peripheral reset
static i2c_status_t i2c_abort_and_reset(i2c_status_t reason)
{
    // Generate STOP
    I2C1->CR1 |= CR1_STOP;

    // Clear NACK flag if set
    if (I2C1->SR1 & I2C_SR1_AF)
        I2C1->SR1 &= ~I2C_SR1_AF;

    // Disable I2C
    I2C1->CR1 &= ~CR1_PE;

    // Small delay to let bus settle
    uint32_t start = millis();
    while (elapsed_ms(start) < 1U) {}

    // Re-enable peripheral
    I2C1->CR1 |= CR1_PE;

    // Check BUSY cleared
    if (wait_flag_clear(&I2C1->SR2, I2C_SR2_BUSY, I2C_BUSY_TIMEOUT_MS) != I2C_OK)
    {
        return I2C_ERR_RECOVERY;
    }

    return reason;
}

// I2C abort and reset if needed
static i2c_status_t wait_flag_set_checked(volatile uint32_t *reg,
                                         uint32_t mask,
                                         uint32_t timeout_ms)
{
    i2c_status_t st = wait_flag_set(reg, mask, timeout_ms);

    if (st != I2C_OK)
        return i2c_abort_and_reset(st);

    return I2C_OK;
}

static i2c_status_t wait_flag_clear_checked(volatile uint32_t *reg,
                                           uint32_t mask,
                                           uint32_t timeout_ms)
{
    i2c_status_t st = wait_flag_clear(reg, mask, timeout_ms);

    if (st != I2C_OK)
        return i2c_abort_and_reset(st);

    return I2C_OK;
}

static i2c_status_t wait_flag_set_or_nack_checked(volatile uint32_t *reg,
                                                 uint32_t mask,
                                                 uint32_t timeout_ms)
{
    i2c_status_t st = wait_flag_set_or_nack(reg, mask, timeout_ms);

    if (st != I2C_OK)
        return i2c_abort_and_reset(st);

    return I2C_OK;
}

