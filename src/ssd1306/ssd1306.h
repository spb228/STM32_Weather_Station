#ifndef SSD1306_H
#define SSD1306_H

#include "common/common_def.h"

// RCC register bit definitions
#define RCC_AHB1ENR_GPIOBEN   (1U << 1)
#define RCC_APB1ENR_I2C1EN    (1U << 21)
#define RCC_APB1RSTR_I2C1RST  (1U << 21)

// GPIO register bit definitions
#define GPIO_MODER_AF         (2U)
#define GPIO_OTYPER_OD        (1U)
#define GPIO_OSPEEDR_HIGH     (3U)
#define GPIO_PUPDR_PU         (1U)
#define GPIO_AF4              (4U)

// I2C register bit definitions
#define I2C_CR1_PE            (1U << 0)
#define I2C_CR1_START         (1U << 8)
#define I2C_CR1_STOP          (1U << 9)
#define I2C_CR1_ACK           (1U << 10)
#define I2C_CR2_FREQ_42MHZ    (42U)
#define I2C_SR1_SB            (1U << 0)
#define I2C_SR1_ADDR          (1U << 1)
#define I2C_SR1_TXE           (1U << 7)
#define I2C_SR1_BTF           (1U << 2)
#define I2C_SR2_MSL           (1U << 0)
#define I2C_SR2_BUSY          (1U << 1)
#define I2C_SR2_TRA           (1U << 2)

// SSD1306 configuration
#define SSD1306_I2C_ADDR      0x3C
#define SSD1306_WIDTH         128
#define SSD1306_HEIGHT        64

// SSD1306 commands
#define SSD1306_DISPLAYOFF    0xAE
#define SSD1306_DISPLAYON     0xAF
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX  0xA8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETSTARTLINE  0x40
#define SSD1306_CHARGEPUMP    0x8D
#define SSD1306_MEMORYMODE    0x20
#define SSD1306_SEGREMAP      0xA0
#define SSD1306_COMSCANDEC    0xC8
#define SSD1306_SETCOMPINS    0xDA
#define SSD1306_SETCONTRAST   0x81
#define SSD1306_SETPRECHARGE  0xD9
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_SETPAGEADDR   0x22
#define SSD1306_SETCOLADDR    0x21

// Data/Command control byte
#define SSD1306_COMMAND       0x00
#define SSD1306_DATA          0x40

// Function prototypes
void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_Display(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y);
void SSD1306_WriteCommand(uint8_t command);
void SSD1306_WriteData(uint8_t* data, uint16_t size);

// I2C interface functions
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_SendAddr(uint8_t addr);
void I2C1_SendByte(uint8_t data);

void delay_ms(uint32_t ms);

#endif /* SSD1306 */