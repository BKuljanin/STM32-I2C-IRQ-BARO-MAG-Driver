#include "stm32f4xx.h"
#define SYSTICK_LOAD_VAL 16000
#define CTRL_ENABLE		(1U<<0)
#define CTRL_CLKSRC		(1U<<2)
#define CTRL_TICKINT	(1U<<1)

static volatile uint32_t timer_ms = 0;

void SysTick_Init(void)
{
	// Systick is used as a heartbeat signal, triggering an interrupt every 1 ms
	// Time it counts to can be configured by changing SYSTICK_LOAD_VAL parameter

	// Cortex m4 generic user guide p249 ch 4.4

	// Reload value for 1 ms. Timer counts down
	SysTick->LOAD = SYSTICK_LOAD_VAL-1; // 16 000 000 / 16 000 = 1000 per second (1ms) which will be timing for timeout

	//Clear systick current register
	SysTick->VAL = 0;

	// Enable systick and select internal clock source
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC; // Cortex m4 generic user guide p249 ch 4.4.1

	// Enable SysTick exception request
	SysTick->CTRL |= CTRL_TICKINT;
}

void SysTick_Handler(void)
{
	// Interrupt occurs every 1ms increasing timer
	timer_ms++;

}

uint32_t millis(void) // Just used to return current time in ms
{
    return timer_ms;
}
