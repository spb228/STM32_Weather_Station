/*
    Main file for the weather station.
*/

#include "common/led/led.h"
#include "common/clock/system_clock.h"
#include "common/systick/systick.h"

int main(void) 
{
    init_led(); 
    sys_clock_config(); // configure system clock to 84MHz
    systick_init();
    uint32_t led_timer = get_tick(); 
    uint32_t led_timeout = 1000; // 1000 milliseconds delay

    while (1)
    {
        if (is_timeout_elapsed(led_timer, led_timeout))
        {
            toggle_led();
            led_timer = get_tick();
        }
    }

    return 0;
}