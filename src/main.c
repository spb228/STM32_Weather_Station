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
    init_led(); 
    sys_clock_config(); // configure system clock to 84MHz
    systick_init(); // initialize systick to count 83999 (1ms for 84MHz clock)
    volatile uint32_t led_timer = get_tick(); // get current tick_time for led toggle. Will be 0 initially.
    volatile uint32_t led_timeout = 1000; // 1000 milliseconds delay

    usart2_config();
    volatile char c = 'a'; 
    
    while (1)
    {
        if (is_timeout_elapsed(led_timer, led_timeout))
        {
            //usart2_send_char(c);
            char str[] = "hello";
            usart2_send_str(str);
            toggle_led();
            led_timer = get_tick();
        }
    }

    return 0;
}