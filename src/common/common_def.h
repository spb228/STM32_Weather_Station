#include <stdint.h>

// System Clock
#define SYSTEM_CLOCK    84000000U   // 84MHz HSE PLL

// Peripheral Address
#define PERIPH_BASE     0x40000000U

// Bus Addresses
#define AHB1_BASE       (PERIPH_BASE + 0x20000U) 

// Clock Addresses
#define RCC_BASE        (AHB1_BASE + 0x3800U)
#define RCC_CR          (* (volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR     (* (volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR        (* (volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR      (*(volatile uint32_t *)(RCC_BASE + 0x40))

// Systick Addresses
#define SYSTICK_BASE    0xE000E010U
#define SYSTICK_CTRL    (* (volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD    (* (volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL     (* (volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SYSTICK_CALIB     (* (volatile uint32_t *)(SYSTICK_BASE + 0x0C))

// GPIO Addresses
#define GPIOA_BASE      (AHB1_BASE + 0x00)
#define GPIOA_MODER     (* (volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       (* (volatile uint32_t *)(GPIOA_BASE + 0x14))

// Power Control Addresses
#define PWR_BASE        0x40007000U
#define PWR_CR          (* (volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CSR         (* (volatile uint32_t *)(PWR_BASE + 0x04))

// Flash Access Control Addresses
#define FLASH_BASE      0x40023C00U
#define FLASH_ACR       (* (volatile uint32_t *)(FLASH_BASE + 0x00))