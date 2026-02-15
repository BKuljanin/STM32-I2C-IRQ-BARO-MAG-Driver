#include <stdio.h>
#include <stdint.h>
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

// bus error related registers
#define SR1_BERR    (1U<<8)   // Bus error
#define SR1_ARLO    (1U<<9)   // Arbitration lost
#define SR1_AF      (1U<<10)  // Acknowledge failure (NACK)
#define SR1_OVR     (1U<<11)  // Overrun/Underrun
#define SR1_PECERR  (1U<<12)  // PEC error (SMBus, usually ignore)
#define SR1_TIMEOUT (1U<<14)  // Timeout/Tlow error
#define SR1_SMBALERT (1U<<15) // SMBus alert (usually ignore)


#define I2C_100KHZ 80 //0B 0101 000 decimal = 80
#define SD_MODE_MAX_RISE_TIME 17 // various tools compute this value

// Timeout constants
#define I2C_FLAG_TIMEOUT_MS   5U
#define I2C_BUSY_TIMEOUT_MS   5U

// Helper functions
#define I2C_SR1_AF (1U<<10)
#define I2C_SR2_BUSY (1U<<1)

// Prototypes of helper functions
static void i2c1_disable_irqs(void);

// Global instance used by IRQ handlers
static i2c_struct g;

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

	GPIOB->OSPEEDR |= (3U<<16) | (3U<<18);  // Reference manual p187. PB8, PB9 high speed

	RCC->APB1ENR |= I2C1EN; // Enable clock access for I2C1, Reference manual p146 bit 21

	// Enter reset mode
	I2C1->CR1 |= (1U<<15); // Reference manual p763, when HIGH, I2C is in reset state

	// Exit reset mode
	I2C1->CR1 &= ~(1U<<15);

	/*Configure peripheral clock*/
	I2C1->CR2 = (1U<<4);// I2C bus frequency. Reference manual p765. 16MHz (10000 binary 5th bit position 5)
	//If clock is 1MHz simply write 1 since its all MHz, set bit 0 to 1; if 2Mhz write to bit 1 (boolean 2)

	I2C1->CR1 |= CR1_ACK;   // Default ACK enabled, will be cleared it for last bytes

	I2C1->CCR = I2C_100KHZ; //CCR bits 0-11 I2C clock. We want standard mode 100kHz, 16MHz/(2*80) = 100kHz

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;// Rise time, reference manual p772,773. The time it takes for the I²C signal (SDA or SCL) to go from LOW to HIGH

	// Enable event + error interrupts
	I2C1->CR2 |= (1U<<9) | (1U<<8) | (1U<<10); // Reference manual p765, 766: ITERR, ITEVT, ITBUF

	NVIC_SetPriority(I2C1_EV_IRQn, 1); // Event interrupt: SB, ADDR, TXE, RXNE, BTF, STOPF
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	NVIC_SetPriority(I2C1_ER_IRQn, 0); // Higher priority than EV
	NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt: AF (acknowledge failure), BERR (bus error), ARLO (arbitration lost), OVR (overrun, underrun), TIMEOUT

	//Enable I2C
	I2C1->CR1 |= CR1_PE;
}


/* Handling timeouts, re-enabling bus, implementing helper function */

// Elapsed time helper function
static inline uint32_t elapsed_ms(uint32_t start)
{
    return (uint32_t)(millis() - start);
}


// Handling interrupts
void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    /* 1. Start condition set */

    // START sent - send address
    if (sr1 & SR1_SB) {
    	 if (g.st == I2C_START) { // Start bit expected
    	            g.addr_is_read = 0;
    	            I2C1->DR = g.saddr <<1; // Reference manual p767. This moves it into bits [7:1] and clears bit 0. R/W 0 = WRITE. So we type slave address, shift one left and that way bit 0 is 0 which is write
    	            //I2C1->DR = (uint8_t)(g.addr7 << 1);       // SLA+W
    	            g.st = I2C_SADDR;
    	        } else if (g.st == I2C_RESTART) { // Repeated start bit expected
    	            g.addr_is_read = 1;
    	            I2C1->DR = g.saddr <<1 | 1;  // Shift slave address and sets bit 0 which is read
    	            g.st = I2C_SADDR;
    	        }
    	 return;
    }


    /* 2. Address sent and ACKed - ADDR */

    if (sr1 & SR1_ADDR) {


        if (g.st ==  I2C_SADDR && g.addr_is_read == 0 && g.operation == 1) {
            // We are in SADDR+W phase of read: next send register address to set internal pointer to where we want to read from
            g.st = I2C_SEND_REG;
        }
        else if (g.st ==  I2C_SADDR && g.addr_is_read == 0 && g.operation == 0) {
            // We are in SADDR+W phase of write: next send register address where reading starts from
        	g.st = I2C_SEND_DATA;
        }
        else if (g.st ==  I2C_SADDR && g.addr_is_read == 1 && g.operation == 1) {
        	// We are in SADDR+R phase of read: read from slave register addressed in SADDR+W phase. This is again after repeated start (after phase 1)
        	g.st = I2C_READ_DATA;
                if (g.n == 1){
                	I2C1->CR1 &= ~CR1_ACK; // Preparing NACK if we read just one byte. It's too late if byte comes to DR and then NACK
                }
                else if (g.n > 1){
                	I2C1->CR1 |= CR1_ACK;	// Preparing ACK for reading bytes when we have several bytes
                }
            }

        volatile uint32_t tmp;
        tmp = I2C1->SR1; // Moving clearing on bottom of the function to be after NACK. If it's before it might happen that we clear ADDR and free the peripheral -->
        tmp = I2C1->SR2; // --> In that case the peripheral is free to proceed and might clock in byte while ACK was still enabled
        (void)tmp; // Just to avoid compiler complaining about not using tmp

        	return;
        }



    /* 3. TXE: DR empty, ready to write */

    if (sr1 & SR1_TXE) { // Previous byte moved from DR to shift register, ready to write

    	if (g.st == I2C_SEND_REG || g.st == I2C_SEND_DATA){

        	if (g.sent_reg == 0) { // First time we enter TXE, both in read and write, we need to write slave register address. Either we read or write starting from that register
        		I2C1->DR = g.maddr;
        		g.sent_reg = 1; // Once we write maddr we set this flag so we don't write slave register address every time TXE triggers, just first time
        		return; // Once we write maddr we stop and next action happens in next TXE interrupt
        	}

            if (g.st == I2C_SEND_REG) { // This operation is a register read. We give repeated start since in previous TXE we wrote maddr. Now repeated start is given
            // Sequence is: START - SLAVE ADDR+W - SLAVE REG ADDR+R - REPEATED START - SLAVE ADDR+R - read bytes
            	I2C1->CR1 |= CR1_START; // During read arms the hardware to generate a repeated START only after the current byte transfer is finished safely.
            	g.st = I2C_RESTART;   // Update state to indicate that the next START generated is the repeated start phase
            }

            else if (g.st == I2C_SEND_DATA) {
            // Now we send data. We wrote maddr where we want to start writing
            	if (g.n > 0 ){
            		I2C1->DR = *g.data++;
            		g.n--;
            		}

            	if (g.n == 0){
            		g.st = I2C_DATA_SENT;
            		}

            	}
    	}
    	return;
        }


    /* 4. RXNE: data received, ready to be read */

    if (sr1 & SR1_RXNE) {
            if (g.st == I2C_READ_DATA) {

            	if(g.n == 1){
            		// In case of receiving only 1 byte, NACK is handled in step 2.

            		// Generate stop condition
            		I2C1->CR1 |= CR1_STOP; // Generate stop after data received to terminate the I2C transaction after the last byte

            		// Store received I2C byte in buffer and increment pointer
            		*g.data++ = I2C1->DR; // data is the pointer to the vector buffer

            		g.st = I2C_IDLE;
            		i2c1_disable_irqs();
            	}

            	else if (g.n > 1){
            		g.n--;
            		*g.data++ = I2C1->DR; // Reading byte from DR
            		if (g.n == 1){
            			I2C1->CR1 &= ~CR1_ACK; // If one more byte remains, set NACK now. It's too late if it gets to DR, so set now while its in shift register
            			// *g.data++ = I2C1->DR; // Reading byte from DR Commented out this is wrong place to read, read hapepns in next interrupt RXNE
            		}
            	}

            }
            return;
        }


    /* 5. BTF: byte transfer finished (important for writing) */
        if (sr1 & SR1_BTF) {
            if (g.st == I2C_DATA_SENT) {
            	// Generate stop condition
            	I2C1->CR1 |= CR1_STOP;
            	g.st = I2C_IDLE;
            	i2c1_disable_irqs();
            }
            return;
        }
}


void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    i2c_status_t st = I2C_ERR_RECOVERY;
    uint8_t need_stop = 1;

    // 1) NACK / acknowledge failure
    // Happens if the slave doesn't ACK address or a data byte.
    if (sr1 & SR1_AF)
    {
        // Clear AF by writing 0 to it
        I2C1->SR1 &= ~SR1_AF;
        st = I2C_ERR_NACK;

        // If we lost arbitration too, ARLO handler below will overwrite st.
        // We generally still try to STOP.
    }

    // 2) Bus error (illegal START/STOP, line glitch, etc.)
    if (sr1 & SR1_BERR)
    {
        I2C1->SR1 &= ~SR1_BERR;
        st = I2C_ERR_RECOVERY;
    }

    // 3) Arbitration lost (multi master or noise)
    // If ARLO happens, we are no longer master; STOP may be meaningless.
    if (sr1 & SR1_ARLO)
    {
        I2C1->SR1 &= ~SR1_ARLO;
        st = I2C_ERR_RECOVERY;

        // After ARLO, hardware switches off master mode. Generating STOP may not do anything.
        // We can avoid forcing STOP to reduce weirdness.
        need_stop = 0;
    }

    // 4) Overrun/Underrun
    // Can happen if DR not read/written in time (often logic bug).
    if (sr1 & SR1_OVR)
    {
        I2C1->SR1 &= ~SR1_OVR;
        st = I2C_ERR_RECOVERY;
    }

    // 5) Timeout SCL low too long etc.
    if (sr1 & SR1_TIMEOUT)
    {
        I2C1->SR1 &= ~SR1_TIMEOUT;
        st = I2C_ERR_TIMEOUT;
    }

    // 6) SMBus-only flags: clear & ignore (unless you intentionally use SMBus)
    if (sr1 & SR1_PECERR)
    {
        I2C1->SR1 &= ~SR1_PECERR;
        // keep existing st
    }

    if (sr1 & SR1_SMBALERT)
    {
        I2C1->SR1 &= ~SR1_SMBALERT;
        // keep existing st
    }

    // If we weren't in an active transfer, just clear flags and exit.
    if (g.st == I2C_IDLE)
        return;

    // Abort/finish: send STOP when it makes sense, then reset state machine
    //i2c1_finish(st, need_stop);
    g.err = st;
    if (need_stop == 1){
    	i2c_abort_and_reset();
    }
}



static void i2c1_disable_irqs(void)
{
    // Disable buffer/event/error interrupts (keep peripheral enabled)
    I2C1->CR2 &= ~((1U<<10) | (1U<<9) | (1U<<8)); // ITBUFEN, ITEVTEN, ITERREN
}


i2c_status_t I2C1_Read(uint8_t saddr,
                       uint8_t maddr,
                       uint8_t *data,
                       uint16_t n)
{
    // Driver busy?
    /*if (g.st != I2C_IDLE) {
        return I2C_BUSY;
    }*/

    // We explicitly forbid 2-byte reads, might need additional configuration
    /*if (len == 2U) {
        return I2C_ERR_UNSUPPORTED;
    }*/

    // Clear completion flag (used by main loop)
    g.done = 0;

    // Fill transfer context
    g.saddr = saddr;
    g.maddr   = maddr;
    g.data = data;
    g.n = n;
    g.operation = I2C_OP_READ;

    // g.status = I2C_OK;

    g.st = I2C_START;

    // Resetting flags
    g.addr_is_read = 0;
    g.sent_reg = 0;

    // Re-enable I2C interrupts
    // ITBUFEN (bit10), ITEVTEN (bit9), ITERREN (bit8)
    I2C1->CR2 |= (1U<<10) | (1U<<9) | (1U<<8);

    // Generate START condition
    // This will cause SB=1 → EV IRQ → SLA+W sent
    I2C1->CR1 |= CR1_START;

    return I2C_OK;
}

i2c_status_t I2C1_Write(uint8_t saddr,
                        uint8_t maddr,
                        uint8_t *data,
                        uint16_t n)
{

    // Clear completion flag (used by main loop)
    g.done = 0;

    // Fill transfer context
    g.saddr = saddr;
    g.maddr   = maddr;
    g.data = data;
    g.n = n;
    g.operation = I2C_OP_WRITE;

    // g.status = I2C_OK;

    g.st = I2C_START;

    // Resetting flags
    g.addr_is_read = 0;
    g.sent_reg = 0;

    // Re-enable I2C interrupts
    // ITBUFEN (bit10), ITEVTEN (bit9), ITERREN (bit8)
    I2C1->CR2 |= (1U<<10) | (1U<<9) | (1U<<8);

    // Generate START condition
    // This will cause SB=1 → EV IRQ → SLA+W sent
    I2C1->CR1 |= CR1_START;

    return I2C_OK;
}



// Abort plus peripheral reset
void i2c_abort_and_reset(void)
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

}

/*static void i2c1_finish(i2c_status_t st, uint8_t send_stop)
{
    // Save status for user/main-loop
    g_last_status = st;

    // If requested, try to generate STOP (safe even if already stopping)
    if (send_stop) {
        I2C1->CR1 |= CR1_STOP;
    }

    // Restore default receive behavior for next transaction
    // (Good hygiene: your RX code may have cleared ACK or set POS.)
    I2C1->CR1 |= CR1_ACK;
    I2C1->CR1 &= ~CR1_POS;

    // Stop further I2C IRQs for this transaction
    i2c1_disable_irq_sources();

    // Mark transfer as finished / driver idle
    g.st = I2C_ST_IDLE;

    // Optional: completion flag for main-loop "wait"
    g_done_flag = 1;
}
*/
