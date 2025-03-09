#include <stdint.h>

// Peripheral Address
#define PERIPH_BASE     0x40000000U

// Bus Addresses
#define AHB1_BASE       (PERIPH_BASE + 0x20000U) 

// Clock Addresses
#define RCC_BASE        (AHB1_BASE + 0x3800U)

// GPIO Addresses
#define GPIOA_BASE      (AHB1_BASE + 0x00U)
#define GPIOA_MODER     (* (volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       (* (volatile uint32_t *)(GPIOA_BASE + 0x14))

// Register Addresses
#define RCC_AHB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x30))