#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include <stdint.h>

// System Clock
#define SYSTEM_CLOCK    84000000U   // 84MHz HSE PLL

// Peripheral Address
#define PERIPH_BASE     0x40000000U

// Bus Addresses
#define APB1_BASE       (PERIPH_BASE + 0x00U)
#define AHB1_BASE       (PERIPH_BASE + 0x20000U) 

// Clock Addresses
#define RCC_BASE        (AHB1_BASE + 0x3800U)
#define RCC_CR          (* (volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR     (* (volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR        (* (volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x40))

// Systick Addresses
#define SYSTICK_BASE    0xE000E010U
#define SYSTICK_CTRL    (* (volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD    (* (volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL     (* (volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SYSTICK_CALIB   (* (volatile uint32_t *)(SYSTICK_BASE + 0x0C))

// GPIO Addresses
#define GPIOA_BASE      (AHB1_BASE + 0x00)
#define GPIOA_MODER     (* (volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER    (* (volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR   (* (volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR     (* (volatile uint32_t *)(GPIOA_BASE + 0x0C))   
#define GPIOA_ODR       (* (volatile uint32_t *)(GPIOA_BASE + 0x14))
#define GPIOA_AFRL      (* (volatile uint32_t *)(GPIOA_BASE + 0x20))

// Power Control Addresses
#define PWR_BASE        0x40007000U
#define PWR_CR          (* (volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CSR         (* (volatile uint32_t *)(PWR_BASE + 0x04))

// Flash Access Control Addresses
#define FLASH_BASE      0x40023C00U
#define FLASH_ACR       (* (volatile uint32_t *)(FLASH_BASE + 0x00))

// USART2 Addresses
#define USART2_BASE     (APB1_BASE + 0x4400)
#define USART2_SR       (* (volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_DR       (* (volatile uint32_t *)(USART2_BASE + 0x04))
#define USART2_BRR      (* (volatile uint32_t *)(USART2_BASE + 0x08))
#define USART2_CR1      (* (volatile uint32_t *)(USART2_BASE + 0x0C))
#define USART2_CR2      (* (volatile uint32_t *)(USART2_BASE + 0x10))
#define USART2_CR3      (* (volatile uint32_t *)(USART2_BASE + 0x14))
#define USART2_GTPR     (* (volatile uint32_t *)(USART2_BASE + 0x18))

// NVIC Addresses
#define NVIC_BASE       (0xE000E000U)
#define NVIC_ISER1      (* (volatile uint32_t *)(NVIC_BASE + 0x104))

#endif // COMMON_DEF_H