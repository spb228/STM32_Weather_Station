#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include <stdint.h>

/* ==================================================================== */
/* System Configuration                                                 */
/* ==================================================================== */

// System Clock
#define SYSTEM_CLOCK    84000000U   // 84MHz HSE PLL

// Peripheral Base Address
#define PERIPH_BASE     0x40000000U

/* ==================================================================== */
/* Bus Bases                                                            */
/* ==================================================================== */

#define APB1_BASE       (PERIPH_BASE + 0x00U)
#define AHB1_BASE       (PERIPH_BASE + 0x20000U) 
#define APB2_BASE       (PERIPH_BASE + 0x10000U)

/* ==================================================================== */
/* RCC (Reset and Clock Control)                                        */
/* ==================================================================== */

#define RCC_BASE        (AHB1_BASE + 0x3800U)
#define RCC_CR          (* (volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR     (* (volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR        (* (volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1RSTR    (* (volatile uint32_t *)(RCC_BASE + 0x20))
#define RCC_APB1ENR     (* (volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR     (* (volatile uint32_t *)(RCC_BASE + 0x44))

/* ==================================================================== */
/* SysTick (System Timer)                                               */
/* ==================================================================== */

#define SYSTICK_BASE    0xE000E010U
#define SYSTICK_CTRL    (* (volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD    (* (volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL     (* (volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SYSTICK_CALIB   (* (volatile uint32_t *)(SYSTICK_BASE + 0x0C))

/* ==================================================================== */
/* GPIO (General Purpose I/O)                                           */
/* ==================================================================== */

// GPIO Port A
#define GPIOA_BASE      (AHB1_BASE + 0x00)
#define GPIOA_MODER     (* (volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER    (* (volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR   (* (volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR     (* (volatile uint32_t *)(GPIOA_BASE + 0x0C))   
#define GPIOA_ODR       (* (volatile uint32_t *)(GPIOA_BASE + 0x14))
#define GPIOA_AFRL      (* (volatile uint32_t *)(GPIOA_BASE + 0x20))

// GPIO Port B
#define GPIOB_BASE      (AHB1_BASE + 0x400)
#define GPIOB_MODER     (* (volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER    (* (volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR   (* (volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR     (* (volatile uint32_t *)(GPIOB_BASE + 0x0C))   
#define GPIOB_AFRL      (* (volatile uint32_t *)(GPIOB_BASE + 0x20))
#define GPIOB_AFRH      (* (volatile uint32_t *)(GPIOB_BASE + 0x24))

/* ==================================================================== */
/* Power & Flash                                                        */
/* ==================================================================== */

// Power Control
#define PWR_BASE        0x40007000U
#define PWR_CR          (* (volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CSR         (* (volatile uint32_t *)(PWR_BASE + 0x04))

// Flash Access Control
#define FLASH_BASE      0x40023C00U
#define FLASH_ACR       (* (volatile uint32_t *)(FLASH_BASE + 0x00))

/* ==================================================================== */
/* USART (Universal Synchronous Asynchronous Receiver Transmitter)      */
/* ==================================================================== */

// USART2
#define USART2_BASE     (APB1_BASE + 0x4400)
#define USART2_SR       (* (volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_DR       (* (volatile uint32_t *)(USART2_BASE + 0x04))
#define USART2_BRR      (* (volatile uint32_t *)(USART2_BASE + 0x08))
#define USART2_CR1      (* (volatile uint32_t *)(USART2_BASE + 0x0C))
#define USART2_CR2      (* (volatile uint32_t *)(USART2_BASE + 0x10))
#define USART2_CR3      (* (volatile uint32_t *)(USART2_BASE + 0x14))
#define USART2_GTPR     (* (volatile uint32_t *)(USART2_BASE + 0x18))

/* ==================================================================== */
/* NVIC (Nested Vectored Interrupt Controller)                          */
/* ==================================================================== */

#define NVIC_BASE       (0xE000E000U)
#define NVIC_ISER0      (* (volatile uint32_t *)(NVIC_BASE + 0x100))
#define NVIC_ISER1      (* (volatile uint32_t *)(NVIC_BASE + 0x104))

/* ==================================================================== */
/* I2C (Inter-Integrated Circuit)                                       */
/* ==================================================================== */

// I2C1
#define I2C1_BASE       (APB1_BASE + 0x5400)
#define I2C1_CR1        (* (volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2        (* (volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1       (* (volatile uint32_t *)(I2C1_BASE + 0x08))
#define I2C1_OAR2       (* (volatile uint32_t *)(I2C1_BASE + 0x0C))
#define I2C1_DR         (* (volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_SR1        (* (volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2        (* (volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_CCR        (* (volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE      (* (volatile uint32_t *)(I2C1_BASE + 0x20))

// I2C2
#define I2C2_BASE       (APB1_BASE + 0x5800)
#define I2C2_CR1        (* (volatile uint32_t *)(I2C2_BASE + 0x00)) // Control Register 1
#define I2C2_CR2        (* (volatile uint32_t *)(I2C2_BASE + 0x04)) // Control Register 2
#define I2C2_OAR1       (* (volatile uint32_t *)(I2C2_BASE + 0x08)) // Own Address Register 1
#define I2C2_OAR2       (* (volatile uint32_t *)(I2C2_BASE + 0x0C)) // Own Address Register 2
#define I2C2_DR         (* (volatile uint32_t *)(I2C2_BASE + 0x10)) // Data Register
#define I2C2_SR1        (* (volatile uint32_t *)(I2C2_BASE + 0x14)) // Status Register 1
#define I2C2_SR2        (* (volatile uint32_t *)(I2C2_BASE + 0x18)) // Status Register 2
#define I2C2_CCR        (* (volatile uint32_t *)(I2C2_BASE + 0x1C)) // Clock Control Register
#define I2C2_TRISE      (* (volatile uint32_t *)(I2C2_BASE + 0x20)) // Rise Time Register
#define I2C2_FLTR       (* (volatile uint32_t *)(I2C2_BASE + 0x24)) // FLTR Register (Noise Filter)

/* ==================================================================== */
/* SPI (Serial Peripheral Interface)                                    */
/* ==================================================================== */

// SPI1
#define SPI1_BASE       (APB2_BASE + 0x3000)
#define SPI1_CR1        (* (volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_CR2        (* (volatile uint32_t *)(SPI1_BASE + 0x04))
#define SPI1_SR         (* (volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR         (* (volatile uint32_t *)(SPI1_BASE + 0x0C))
#define SPI1_CRCPR      (* (volatile uint32_t *)(SPI1_BASE + 0x10))
#define SPI1_RXCRCR     (* (volatile uint32_t *)(SPI1_BASE + 0x14))
#define SPI1_TXCRCR     (* (volatile uint32_t *)(SPI1_BASE + 0x18))

/* ==================================================================== */
/* Helper Bit Definitions (Reference: RM0368)                           */
/* ==================================================================== */

// GPIO Config Macros
#define GPIO_MODER_INPUT     0x00
#define GPIO_MODER_OUTPUT    0x01
#define GPIO_MODER_AF        0x02
#define GPIO_MODER_ANALOG    0x03

#define GPIO_OTYPER_PP       0x00 // Push-Pull
#define GPIO_OTYPER_OD       0x01 // Open-Drain

#define GPIO_OSPEEDR_LOW     0x00
#define GPIO_OSPEEDR_MED     0x01
#define GPIO_OSPEEDR_HIGH    0x02
#define GPIO_OSPEEDR_VHIGH   0x03

#define GPIO_PUPDR_NONE      0x00
#define GPIO_PUPDR_PU        0x01
#define GPIO_PUPDR_PD        0x02

#define GPIO_AF4             0x04 // Alternate Function 4 (I2C1/2/3)
#define GPIO_AF9             0x09 // Alternate Function 9 (I2C2)

// I2C Control Bits
#define I2C_CR1_PE           (1U << 0)  // Peripheral Enable
#define I2C_CR1_START        (1U << 8)  // Start Generation
#define I2C_CR1_STOP         (1U << 9)  // Stop Generation
#define I2C_CR1_ACK          (1U << 10) // Acknowledge Enable

#define I2C_SR1_SB           (1U << 0)  // Start Bit (Master mode)
#define I2C_SR1_ADDR         (1U << 1)  // Address sent (Master mode)
#define I2C_SR1_BTF          (1U << 2)  // Byte Transfer Finished
#define I2C_SR1_RXNE         (1U << 6)  // Data Register not empty (Receiver)
#define I2C_SR1_TXE          (1U << 7)  // Data Register empty (Transmitter)

#define I2C_SR2_BUSY         (1U << 1)  // Bus Busy


#endif /* COMMON_DEF_H */