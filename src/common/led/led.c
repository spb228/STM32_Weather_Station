#include "led.h"
#include "common/uart/uart.h"

void init_led(void)
{
    usart2_send_str("[led.c] [DBG] Initializing LED\n");
    RCC_AHB1ENR |= (1 << 0); // enable GPIOA
    GPIOA_MODER &= ~(3 << (LED_PIN * 2)); // clear bit 10 and 11
    GPIOA_MODER |= (1 << (LED_PIN * 2)); // set bit 10 for output mode
}

void led_delay(uint32_t delay)
{
    while(delay--)
    {
        __asm("nop");
    }
}

void toggle_led(void)
{
    usart2_send_str("[led.c] [DBG] Toggling LED\n");
    GPIOA_ODR ^= (1 << LED_PIN);
}