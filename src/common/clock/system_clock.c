#include "system_clock.h"

void init_sys_clock(void)
{
    RCC_CR |= (1 << 16); // set HSEON (high speed enable)
    while (!(RCC_CR & (1 << 17))); // wait till HSE ready flag is set

    FLASH_ACR = (2 << 0) |  // two wait state latency
                (1 << 8) |  // prefetch enable
                (1 << 9) |  // instruction cache enable
                (1 << 10);  // data cache enable

    RCC_PLLCFGR = 0; 
    RCC_PLLCFGR |= (1 << 22)  |      // PLLSRC = HSE source
                   (8 << 0)   |      // PLLM = 8
                   (336 << 6) |      // PLLN = 336
                   (1 << 16)  |      // PLLP = 4 (encoded as 1)
                   (7 << 24);        // PLLQ = 7 (48MHz for USB)

    RCC_CR |= (1 << 24);             // PLLON set
    while (!(RCC_CR & (1 << 25)));   // PLL is locked and ready

    RCC_CFGR = 0;
    RCC_CFGR |= (0 << 4)  |          // AHB prescaler = 1 (SYSCLK not divided)
                //(5 << 10) |        // APB1 prescaler = 4 (HCLK divided by 4 -> 21MHz)
                (4 << 10);           // APB1 prescaler = 2 (HCLK divided by 4 -> 42MHz)
                //(4 << 13);         // APB2 prescaler = 2 (HCLK divided by 2 -> 42MHz)
    RCC_CFGR &= ~(7 << 13);          // APB2 prescaler = 0 (HCLK not divided  -> 84MHz)   

    RCC_CFGR |= (2 << 0);            // PLL used as sys clock
    while ((RCC_CFGR & (3 << 2)) != (2 << 2)); // wait until PLL is being used
}

void sys_clock_config(void)
{
    RCC_APB1ENR |= (1 << 28);       // PWR clock enable
    PWR_CR |= (3 << 14);            // VOS = 3 (Scale 1 mode, highest performance)

    init_sys_clock();
}