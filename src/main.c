/*
    Main file for the weather station.
*/
#include <string.h>
#include "common/led/led.h"
#include "common/clock/system_clock.h"
#include "common/systick/systick.h"
#include "common/uart/uart.h"

int main(void)
{
    sys_clock_config(); // configure system clock to 84MHz
    usart2_config();    // initialize UART pins and configs
    init_led();         // initialize LED pins and configs
    systick_init();     // initialize systick to count 83999 (1ms for 84MHz clock)

    __asm volatile ("cpsie i"); // enable ext interrupts

    volatile uint32_t led_timer = get_tick(); // get current tick_time for led toggle. Will be 0 initially.
    volatile uint32_t led_timeout = 1000;     // blink LED every 1000 seconds

    while (1)
    {
        if (is_timeout_elapsed(led_timer, led_timeout))
        {
            usart2_send_str("Test"); 
            toggle_led();
            led_timer = get_tick();
        }
    }

    return 0;
}