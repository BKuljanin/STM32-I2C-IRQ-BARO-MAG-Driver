#include "uart.h"

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ
#define UART_BAUDRATE 115200

#define GPIOAEN	(1U<<0)
#define UART2EN (1U<<17)
#define CR1_TE (1U<<3)
#define CR1_RE (1U<<2)
#define CR1_UE (1U<<13)
#define SR_TXE (1U<<7)
#define SR_RXE (1U<<5)

#define CR1_RXNEIE (1U<<5)

#define DMA1EN (1U<<21)
#define CHSEL4 (1U<<27)
#define DMA_MEM_INC (1U<<10)
#define DMA_DIR_MEM_TO_PERIPH (1U<<6)
#define DMA_CR_TCIE (1U<<4)
#define DMA_CR_EN (1U<<0)
#define UART_CR3_DMAT (1U<<7)

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void uart2_write(int ch);
void uart2_tx_init(void);

// new function for DMA
void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len) //arugments: source, destination and length of the data
{
	// Enable clock access to DMA
	RCC->AHB1ENR |= DMA1EN;// datasheet p16 shows AHB1 bus connected to DMA1/2. Ref manual p137 bit 21

	// Disable DMA1 stream6
	DMA1_Stream6->CR &=~ DMA_CR_EN; // reference manual p226 DMA stream configuration register, we need to disable bit 0

	// Wait until DMA1 stream 6 is disabled
	while(DMA1_Stream6->CR & DMA_CR_EN){} // if returns true we are stuck here until its disabled

	// Clear interrupt flags for stream 6
	DMA1->HIFCR |= (1U<<16); // Reference manual p225, high interrupt flag clear register. Register starts with stream 4, then 5,6,7 (thats why its high reg, low starts with 0)
	DMA1->HIFCR |= (1U<<18); // this is all flag clear register, these bits are for stream 6 (clears bits by putting 1 in them)
	DMA1->HIFCR |= (1U<<19);
	DMA1->HIFCR |= (1U<<20);
	DMA1->HIFCR |= (1U<<21);

	// Set the destination buffer
	DMA1_Stream6->PAR = dst; // Destination that the user will pass

	// Set the source buffer
	DMA1_Stream6->M0AR = src;// Memory source. Stream peripheral address register. Reference manual p229. Source is memory. Memory 0 AR

	// Set length
	DMA1_Stream6-> NDTR = len; // Register hold length of the trasnfer

	// Select stream 6 channel 4
	DMA1_Stream6->CR = CHSEL4; // Reference manual p226 stream configuration register. CHSEL bit is for channel select. Bits 25,26,27. Clear register except for 27. 100 is channel 4

	// Enable memory increment. Configure memory to increment because the data is stored in buffer, when you access first index of the buffer you want to increment next index
	DMA1_Stream6->CR |= DMA_MEM_INC;// Same register as above, MINC (memory increment). This time we put OR because now we cant clear entire register, we just want to add this one bit

	// Configure transfer direction (memory->peripheral). Move content of the array and move it to the UART periph.
	DMA1_Stream6->CR |= DMA_DIR_MEM_TO_PERIPH;// Same register as above, bits 6,7 DIR. 00 periph to memory, 01 mem to periph, 10 mem-to-mem. We want 01 memory to peripheral, so we just set one bit

	// Enable DMA transfer complete interrupt
	DMA1_Stream6->CR |= DMA_CR_TCIE; // We will use only transfer complete interrupt. Reference manual p226 DMA stream configuration register. Transfer complete interrupt enable, TCIE bit 4

	// Enable direct mode and disable FIFO
	DMA1_Stream6->FCR = 0; // Fifo control register, reference manual p231. 0 in the register is interrupt disabled, also 0 is direct mode enable

	// Enable DMA1 stream 6
	DMA1_Stream6->CR |= DMA_CR_EN; // DMA stream configuration register p226 ref man

	// Enable UART2 transmitter DMA
	USART2->CR3 |= UART_CR3_DMAT; // This is configured in uart register. Ref man p822 bit 7 DMA for transmitter

	// Enable DMA interrupt in NVIC. After transfer is completed decide to run a piece of code (or in the middle of transfer or complete etc)
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}


void uart2_rx_interrupt_init(void)
{
	// obsolete tx lines can be deleted from here, used as rx only
	RCC->AHB1ENR |= GPIOAEN;


	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |= (1U<<5);


	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	// Now we also need to configure PA3 in order to have RX, bits 6 and 7 for PA3
	GPIOA->MODER &=~ (1U<<6);
	GPIOA->MODER |= (1U<<7);
	// GPIO alternate function register p190 reference manual
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &=~ (1U<<15); // now GPIO is configured for UART



	RCC->APB1ENR |=  UART2EN; // connected to APB1, so we enable APB1


	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	// Here we enabled transmission. Now we need to enable receiver. Reference manual p818 bit 2 is RE
	//USART2->CR1 = CR1_TE;
	USART2->CR1 = CR1_TE | CR1_RE; // Now we enable these two bits at once with OR.

	USART2->CR1 |= CR1_RXNEIE; // New line: enabling interrupt in UART control register! (RXNE interrupt)

	NVIC_EnableIRQ(USART2_IRQn); // New line: Enable UART2 interrupt in NVIC!

	// Enable UART module
	USART2->CR1 |= CR1_UE;

}

void uart2_rxtx_init(void)
{

	RCC->AHB1ENR |= GPIOAEN;


	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |= (1U<<5);


	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	// Now we also need to configure PA3 in order to have RX, bits 6 and 7 for PA3
	GPIOA->MODER &=~ (1U<<6);
	GPIOA->MODER |= (1U<<7);
	// GPIO alternate function register p190 reference manual
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &=~ (1U<<15); // now GPIO is configured for UART



	RCC->APB1ENR |=  UART2EN; // connected to APB1, so we enable APB1


	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	// Here we enabled transmission. Now we need to enable receiver. Reference manual p818 bit 2 is RE
	//USART2->CR1 = CR1_TE;
	USART2->CR1 = CR1_TE | CR1_RE; // Now we enable these two bits at once with OR.



	// Enable UART module
	USART2->CR1 |= CR1_UE;

}


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}


static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return((PeriphClk + (BaudRate/2U))/BaudRate);
}


void uart2_write(int ch)
{

	while(!(USART2->SR & SR_TXE)){}
	// Write to transmit data register
	USART2->DR = (ch & 0xFF);
}


// Now we also need a function to read from UART
char uart2_read(void)
{
	// Make sure that receive data register is NOT EMPTY.
	while(!(USART2->SR & SR_RXE)){}; //The same as we did for transmitting, except now we are receiving. Reference manual p815 status register. If this is empty there is nothing to read!
	return USART2->DR;
}






// NOTE: The default clock time if not configured should be 16MHz. However for some reason it wasn't the default value. The next function's purpose is to set the clock to 16MHz
void clock_16MHz(void)
{
    // Enable HSI (bit 0 in RCC_CR)
    RCC->CR |= RCC_CR_HSION;

    // Wait until HSI is ready
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Select HSI as system clock (bits 1:0 = 00 in RCC_CFGR)
    RCC->CFGR &= ~(3U << 0);

    // Wait until HSI is used as system clock
    while ((RCC->CFGR & (3U << 2)) != 0);
}



void uart2_tx_init(void)
{

	RCC->AHB1ENR |= GPIOAEN;


	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |= (1U<<5);


	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);


	// Configure UART module

	// Enable clock access to UART2
	RCC->APB1ENR |=  UART2EN; // connected to APB1, so we enable APB1

	// Configure UART baudrate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	// Configure the transfer direction, this is just TX. Reference manual p818
	USART2->CR1 = CR1_TE; // we want to clean everything in uart and just set bit to 1 and everything else 0. This is default uart. The rest of registers from p823 are default for usual UART comm.

	// Enable UART module
	USART2->CR1 |= CR1_UE; // enabling the USART, bit 13 p819. Or operator because its same CR1, we just add bit to current state, we dont want to clean it. We already configured a bit before


}


int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}


