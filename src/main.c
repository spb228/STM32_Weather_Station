/*
    Main file for the weather station.
*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "common/led/led.h"
#include "common/clock/system_clock.h"
#include "common/systick/systick.h"
#include "common/uart/uart.h"
#include "aht10/aht10.h"
#include "ssd1306/ssd1306.h"
#include "usd_card/usd_card.h"

#define LED_TIMEOUT             5000 // blink LED every 5 seconds

void init_display(void)
{
    SSD1306_Wrapper();  // initialize I2C1 init and SSD1306 OLED display //
}

void init_temp_humid_sensor(aht10_t* temp_humid_sensor)
{
    aht10_init(temp_humid_sensor);
}

void init_sd_card(void)
{
    SPI1_Init();        // initialize SPI1 for SD Card
    SD_Init();          // initialize SD Card

    if (SD_Send_CMD8() != 0)
    {
        print("CMD8 failed\r\n");
        while (1); // TODO: handle error gracefully
    }
}

int main(void)
{
    /* Core System init */
    sys_clock_config(); // configure system clock to 84MHz
    usart2_config();    // initialize UART pins and configs
    init_led();         // initialize LED pins and configs
    systick_init();     // initialize systick to count 83999 (1ms for 84MHz clock)

    /* Peripherals init */
    init_display();
    aht10_t temp_humid_sensor; 
    init_temp_humid_sensor(&temp_humid_sensor);
    init_sd_card();

    __asm volatile ("cpsie i"); // enable ext interrupts

    volatile uint32_t led_timer = get_tick(); // get current tick_time for led toggle. Will be 0 initially.

    while (1)
    {
        if (is_timeout_elapsed(led_timer, LED_TIMEOUT))
        {
            toggle_led();

            /* Get temp/humidity data */
            aht10_run_cycle(&temp_humid_sensor);
            float t = temp_humid_sensor.temp_c; 
            float h = temp_humid_sensor.humid_pct;

            /* TODO: send data to the oled display */

            /* TODO: send data to the SD card */

            led_timer = get_tick();
        }
    }

    return 0;
}