/*
    Main file for the weather station.
*/

#include "common/led/led.h"

int main(void) 
{
    init_led(); 
    uint32_t led_delay_val = 4000000; // 4 million cycles

    while (1)
    {
        toggle_led();
        led_delay(led_delay_val);
    }

    return 0;
}