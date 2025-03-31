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

#define LED_TIMEOUT     1000 // blink LED every 1 seconds

// Simple pattern function to test the display
void draw_test_pattern(void) 
{
    // Draw a border around the display
    for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
        SSD1306_DrawPixel(x, 0);
        SSD1306_DrawPixel(x, SSD1306_HEIGHT - 1);
    }
    for (uint8_t y = 0; y < SSD1306_HEIGHT; y++) {
        SSD1306_DrawPixel(0, y);
        SSD1306_DrawPixel(SSD1306_WIDTH - 1, y);
    }
    
    // Draw a diagonal line
    for (uint8_t i = 0; i < 64; i++) {
        SSD1306_DrawPixel(i, i);
        SSD1306_DrawPixel(i + 64, 63 - i);
    }
    
    // Draw a cross in the middle
    for (uint8_t i = 0; i < SSD1306_WIDTH; i++) {
        SSD1306_DrawPixel(i, SSD1306_HEIGHT / 2);
    }
    for (uint8_t i = 0; i < SSD1306_HEIGHT; i++) {
        SSD1306_DrawPixel(SSD1306_WIDTH / 2, i);
    }
}

int main(void)
{
    sys_clock_config(); // configure system clock to 84MHz
    usart2_config();    // initialize UART pins and configs
    init_led();         // initialize LED pins and configs
    systick_init();     // initialize systick to count 83999 (1ms for 84MHz clock)

    //__asm volatile ("cpsie i"); // enable ext interrupts

    volatile uint32_t led_timer = get_tick(); // get current tick_time for led toggle. Will be 0 initially.

    // Initialize the SSD1306 OLED display
    SSD1306_Init();
    
    // Clear the display
    SSD1306_Clear();
    
    // Draw a test pattern
    draw_test_pattern();
    
    // Update the display with the buffer content
    SSD1306_Display();
 
    while (1)
    {
        if (is_timeout_elapsed(led_timer, LED_TIMEOUT))
        {
            print("blinking LED...\r\n"); 
            toggle_led();
            led_timer = get_tick();
        }
    }

    return 0;
}