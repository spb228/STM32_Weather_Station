#include "led.h"

void init_led()
{
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

void toggle_led()
{
    GPIOA_ODR ^= (1 << LED_PIN);
}