#include "systick.h"
#include "common/uart/uart.h"

volatile uint32_t g_tick_count = 0; // global tick variable for systick

void systick_init(void)
{
    usart2_send_str("[systick.c] [DBG] Initializing systick configs\n");
    SYSTICK_CTRL = 0; // disable systick timer
    SYSTICK_LOAD = (SYSTEM_CLOCK / 1000) - 1; // for 84MHz this would be 83999
    SYSTICK_VAL = 0; // clear current value
    SYSTICK_CTRL |= (1 << 2) | // clock source: 0 = AHB/8, 1 = processor clock (AHB) 
                    (1 << 1) | // enable interrupt when counter reaches 0
                    (1 << 0);  // enable counter
}

void SysTick_Handler(void)
{
    g_tick_count++; 
}

uint32_t get_tick(void)
{
    return g_tick_count; 
}

uint8_t is_timeout_elapsed(uint32_t start_time, uint32_t timeout) 
{
    return (uint8_t)((get_tick() - start_time) >= timeout);
}

