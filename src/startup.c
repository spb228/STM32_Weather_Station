#include <stdint.h>

// Forward declarations
void Reset_Handler(void);
void Default_Handler(void);
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak,alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

extern int main(void);

// Linker script symbols
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

#define STACK_START 0x20018000U // End of RAM for STM32F401RE

// Vector table
uint32_t vectors[] __attribute__((section(".isr_vector"), aligned(0x80))) = {
    STACK_START,                // Stack pointer
    (uint32_t)Reset_Handler,    // Reset
    (uint32_t)Default_Handler,  // NMI
    (uint32_t)Default_Handler,  // Hard Fault
    (uint32_t)Default_Handler,  // MemManage
    (uint32_t)Default_Handler,  // BusFault
    (uint32_t)Default_Handler,  // UsageFault
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    (uint32_t)Default_Handler,  // SVCall
    (uint32_t)Default_Handler,  // Debug Monitor
    0,                          // Reserved
    (uint32_t)Default_Handler,  // PendSV
    (uint32_t)SysTick_Handler,  // SysTick
    // External interrupts (IRQs)
    (uint32_t)Default_Handler,  // 0: Window Watchdog
    (uint32_t)Default_Handler,  // 1: PVD through EXTI Line detection
    (uint32_t)Default_Handler,  // 2: Tamper and TimeStamp
    (uint32_t)Default_Handler,  // 3: RTC Wakeup
    (uint32_t)Default_Handler,  // 4: Flash
    (uint32_t)Default_Handler,  // 5: RCC
    (uint32_t)Default_Handler,  // 6: EXTI Line0
    (uint32_t)Default_Handler,  // 7: EXTI Line1
    (uint32_t)Default_Handler,  // 8: EXTI Line2
    (uint32_t)Default_Handler,  // 9: EXTI Line3
    (uint32_t)Default_Handler,  // 10: EXTI Line4
    (uint32_t)Default_Handler,  // 11: DMA1 Stream 0
    (uint32_t)Default_Handler,  // 12: DMA1 Stream 1
    (uint32_t)Default_Handler,  // 13: DMA1 Stream 2
    (uint32_t)Default_Handler,  // 14: DMA1 Stream 3
    (uint32_t)Default_Handler,  // 15: DMA1 Stream 4
    (uint32_t)Default_Handler,  // 16: DMA1 Stream 5
    (uint32_t)Default_Handler,  // 17: DMA1 Stream 6
    (uint32_t)Default_Handler,  // 18: ADC1, ADC2, ADC3
    (uint32_t)Default_Handler,  // 19: CAN1 TX
    (uint32_t)Default_Handler,  // 20: CAN1 RX0
    (uint32_t)Default_Handler,  // 21: CAN1 RX1
    (uint32_t)Default_Handler,  // 22: CAN1 SCE
    (uint32_t)Default_Handler,  // 23: EXTI Lines 9:5
    (uint32_t)Default_Handler,  // 24: TIM1 Break and TIM9
    (uint32_t)Default_Handler,  // 25: TIM1 Update and TIM10
    (uint32_t)Default_Handler,  // 26: TIM1 Trigger and Commutation and TIM11
    (uint32_t)Default_Handler,  // 27: TIM1 Capture Compare
    (uint32_t)Default_Handler,  // 28: TIM2
    (uint32_t)Default_Handler,  // 29: TIM3
    (uint32_t)Default_Handler,  // 30: TIM4
    (uint32_t)I2C1_EV_IRQHandler,  // 31: I2C1 Event
    (uint32_t)I2C1_ER_IRQHandler,  // 32: I2C1 Error
    (uint32_t)Default_Handler,  // 33: I2C2 Event
    (uint32_t)Default_Handler,  // 34: I2C2 Error
    (uint32_t)Default_Handler,  // 35: SPI1
    (uint32_t)Default_Handler,  // 36: SPI2
    (uint32_t)Default_Handler,  // 37: USART1
    (uint32_t)USART2_IRQHandler // 38: USART2
};

// Default handler for interrupts
void Default_Handler(void)
{
    while (1)
    {
    }
}

// Reset handler
void Reset_Handler(void)
{
    // Copy data from FLASH to RAM
    uint32_t *pSrc = &_etext;
    uint32_t *pDest = &_sdata;
    while (pDest < &_edata)
    {
        *pDest++ = *pSrc++;
    }

    // Zero fill BSS
    uint32_t *pBss = &_sbss;
    while (pBss < &_ebss)
    {
        *pBss++ = 0;
    }

    // Call main
    main();

    // Loop forever if main returns
    while (1)
    {
    }
}