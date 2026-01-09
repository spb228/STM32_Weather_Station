/*
    Main file for the weather station.
*/
#include <stdlib.h>

#include <string.h>
#include "common/led/led.h"
#include "common/clock/system_clock.h"
#include "common/systick/systick.h"
#include "common/uart/uart.h"
#include "aht10/aht10.h"
#include "ssd1306/ssd1306.h"
#include "usd_card/usd_card.h"

#define LED_TIMEOUT     1000 // blink LED every 1 seconds

int main(void)
{
    sys_clock_config(); // configure system clock to 84MHz
    usart2_config();    // initialize UART pins and configs
    init_led();         // initialize LED pins and configs
    systick_init();     // initialize systick to count 83999 (1ms for 84MHz clock)
    SSD1306_Wrapper();  // Run the SSD1306 OLED display //

    SPI1_GPIO_Init();
    SPI1_Init();
    SD_Init();

    if (SD_Send_CMD8() != 0)
    {
        print("CMD8 failed\r\n");
        while (1);
    }

    __asm volatile ("cpsie i"); // enable ext interrupts

    volatile uint32_t led_timer = get_tick(); // get current tick_time for led toggle. Will be 0 initially.

    while (1)
    {
        if (is_timeout_elapsed(led_timer, LED_TIMEOUT))
        {
            print("blinking LED...\r\n"); 
            toggle_led();
            led_timer = get_tick();
        }

        //TODO :  add aht10, ssd1306, and SD card code here after testing...

    }

    return 0;
}