# STM32F401RE Weather Station Data Logger Blueprint

## Hardware Components
- **Microcontroller**: STM32F401RE Nucleo Dev Board
- **Display**: SSD1306 (I2C)
- **Temp/Humidity Sensor**: AHT10 (I2C)
- **MicroSD adapter/module** (SPI) for data storage

## System Architecture

### Hardware Interfaces
- STM32F401RE ↔ SSD1306 Display (I2C1)
- STM32F401RE ↔ AHT10 Sensor (I2C1, shared bus)
- STM32F401RE ↔ MicroSD Card (SPI1)

### Software Layers
1. **Hardware Abstraction Layer (HAL)**
   - Direct register manipulation for each peripheral
   - No standard libraries/HALs to maintain bare metal approach

2. **Driver Layer**
   - I2C driver for display and sensor
   - SPI driver for SD card
   - FatFS implementation for file system

3. **Application Layer**
   - Sensor data acquisition
   - Display controller
   - Data logging subsystem
   - Real-time clock management

## Implementation Roadmap

### 1. System Initialization
- Configure system clock (84MHz from 8MHz HSE)
- Initialize GPIO for status LEDs
- Configure I2C1 for sensor and display
- Configure SPI1 for SD card

### 2. Peripheral Drivers Development

#### I2C Driver
```c
// i2c.h
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

// I2C status codes
typedef enum {
    I2C_OK = 0,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_ARBITRATION_LOST,
    I2C_ERROR_NACK,
    I2C_ERROR_BUS
} I2C_Status;

// Initialize I2C peripheral
void I2C_Init(void);

// Transmit data to slave device
I2C_Status I2C_TransmitData(uint8_t slave_addr, uint8_t* data, uint16_t size);

// Receive data from slave device
I2C_Status I2C_ReceiveData(uint8_t slave_addr, uint8_t* data, uint16_t size);

// Transmit and then receive (for register access)
I2C_Status I2C_TransmitReceiveData(uint8_t slave_addr, 
                                    uint8_t* tx_data, uint16_t tx_size, 
                                    uint8_t* rx_data, uint16_t rx_size);

#endif // I2C_H
```

```c
// i2c.c
#include "i2c.h"
#include "stm32f401xe.h" // Device header for register definitions

// Initialize I2C in master mode (100kHz standard mode)
void I2C_Init(void) {
    // Enable clock for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // Configure PB8 (SCL) and PB9 (SDA) as alternate function
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); // Alternate function
    
    // Set AF4 (I2C1) for PB8 and PB9
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk);
    GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos));
    
    // Set open-drain output type
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    
    // Enable pull-up resistors
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0); // Pull-up
    
    // Configure I2C1
    I2C1->CR1 = 0; // Reset I2C1 control register 1
    
    // Set I2C clock frequency in CCR
    // For 100 kHz from 84 MHz APB1 clock:
    // CCR = APB1_CLK / (I2C_CLK * 2) = 84MHz / (100kHz * 2) = 420
    I2C1->CCR = 420;
    
    // Set rise time register
    // For 100 kHz, rise time is 1000ns = 1us
    // TRISE = (1us / (1/84MHz)) + 1 = 84 + 1 = 85
    I2C1->TRISE = 85;
    
    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

// Wait for specified I2C event with timeout
static I2C_Status I2C_WaitEvent(uint32_t event, uint32_t timeout) {
    uint32_t start_time = /* Get current tick */;
    
    while (!(I2C1->SR1 & event)) {
        if (/* Current tick */ - start_time > timeout) {
            return I2C_ERROR_TIMEOUT;
        }
    }
    
    return I2C_OK;
}

I2C_Status I2C_TransmitData(uint8_t slave_addr, uint8_t* data, uint16_t size) {
    // Generate START condition
    I2C1->CR1 |= I2C_CR1_START;
    
    // Wait for start condition to be generated
    if (I2C_WaitEvent(I2C_SR1_SB, 1000) != I2C_OK) {
        return I2C_ERROR_TIMEOUT;
    }
    
    // Send slave address with write bit (0)
    I2C1->DR = slave_addr << 1;
    
    // Wait for address to be sent
    if (I2C_WaitEvent(I2C_SR1_ADDR, 1000) != I2C_OK) {
        return I2C_ERROR_TIMEOUT;
    }
    
    // Clear ADDR flag by reading SR2
    (void)I2C1->SR2;
    
    // Send data byte by byte
    for (uint16_t i = 0; i < size; i++) {
        I2C1->DR = data[i];
        
        // Wait for byte to be transmitted
        if (I2C_WaitEvent(I2C_SR1_TXE, 1000) != I2C_OK) {
            return I2C_ERROR_TIMEOUT;
        }
    }
    
    // Wait for transmission to complete
    if (I2C_WaitEvent(I2C_SR1_BTF, 1000) != I2C_OK) {
        return I2C_ERROR_TIMEOUT;
    }
    
    // Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;
    
    return I2C_OK;
}

I2C_Status I2C_ReceiveData(uint8_t slave_addr, uint8_t* data, uint16_t size) {
    // Implementation details similar to TransmitData but for receiving
    // Not fully implemented in this blueprint
    return I2C_OK;
}

I2C_Status I2C_TransmitReceiveData(uint8_t slave_addr, 
                                   uint8_t* tx_data, uint16_t tx_size, 
                                   uint8_t* rx_data, uint16_t rx_size) {
    // Implementation details for combined operations
    // Not fully implemented in this blueprint
    return I2C_OK;
}
```

#### SPI Driver
```c
// spi.h
#ifndef SPI_H
#define SPI_H

#include <stdint.h>

// SPI status codes
typedef enum {
    SPI_OK = 0,
    SPI_ERROR_TIMEOUT,
    SPI_ERROR_OVERRUN
} SPI_Status;

// Initialize SPI peripheral
void SPI_Init(void);

// Set chip select pin (active low)
void SPI_ChipSelect(uint8_t state);

// Transmit/receive a single byte
uint8_t SPI_TransmitReceiveByte(uint8_t byte);

// Transmit multiple bytes
SPI_Status SPI_TransmitData(uint8_t* data, uint16_t size);

// Receive multiple bytes
SPI_Status SPI_ReceiveData(uint8_t* data, uint16_t size);

#endif // SPI_H
```

```c
// spi.c
#include "spi.h"
#include "stm32f401xe.h" // Device header for register definitions

// Initialize SPI for SD card communication
void SPI_Init(void) {
    // Enable clock for GPIOA and SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Alternate function
    
    // Set AF5 (SPI1) for PA5, PA6, PA7
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk);
    GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos));
    
    // Configure PA4 as GPIO output for chip select
    GPIOA->MODER &= ~GPIO_MODER_MODER4_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER4_0; // Output mode
    GPIOA->BSRR = GPIO_BSRR_BS4; // Set high (inactive)
    
    // Configure SPI1 for:
    // - Master mode
    // - Clock polarity low (CPOL=0)
    // - Clock phase 1st edge (CPHA=0)
    // - 8-bit data frame
    // - MSB first
    // - NSS managed by software
    // - Prescaler = 256 for low speed initialization (~328kHz)
    
    SPI1->CR1 = 0;
    SPI1->CR1 |= (SPI_CR1_MSTR |          // Master mode
                 SPI_CR1_SSM | SPI_CR1_SSI | // Software slave management
                 (7 << SPI_CR1_BR_Pos));  // Prescaler 256
    
    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI_ChipSelect(uint8_t state) {
    if (state == 0) {
        // Active low, pull CS pin low
        GPIOA->BSRR = GPIO_BSRR_BR4;
    } else {
        // Pull CS pin high
        GPIOA->BSRR = GPIO_BSRR_BS4;
    }
}

uint8_t SPI_TransmitReceiveByte(uint8_t byte) {
    // Wait for TXE (Transmit buffer empty)
    while (!(SPI1->SR & SPI_SR_TXE));
    
    // Send byte
    SPI1->DR = byte;
    
    // Wait for RXNE (Receive buffer not empty)
    while (!(SPI1->SR & SPI_SR_RXNE));
    
    // Return received byte
    return SPI1->DR;
}

SPI_Status SPI_TransmitData(uint8_t* data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        SPI_TransmitReceiveByte(data[i]);
    }
    return SPI_OK;
}

SPI_Status SPI_ReceiveData(uint8_t* data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        data[i] = SPI_TransmitReceiveByte(0xFF); // Send dummy byte to receive data
    }
    return SPI_OK;
}
```

### 3. Device Drivers

#### SSD1306 Display Driver
```c
// ssd1306.h
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>

// Display dimensions
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

// I2C address (typically 0x3C or 0x3D)
#define SSD1306_I2C_ADDR 0x3C

// Initialize SSD1306 OLED display
bool SSD1306_Init(void);

// Clear display buffer
void SSD1306_Clear(void);

// Update display with current buffer
bool SSD1306_Display(void);

// Draw pixel at specific coordinates
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

// Draw text at specific coordinates
void SSD1306_DrawText(uint8_t x, uint8_t y, const char* text);

// Draw simple graph (for temperature/humidity trending)
void SSD1306_DrawGraph(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float* data, uint8_t data_len);

#endif // SSD1306_H
```

```c
// ssd1306.c
#include "ssd1306.h"
#include "i2c.h"
#include <string.h>

// Display buffer - 1024 bytes for 128x64 display (1 bit per pixel)
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Font data (5x7 font)
static const uint8_t Font5x7[] = {
    // Font data would go here (not fully included in blueprint)
    0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    // ... more font data ...
};

// Send command to SSD1306
static bool SSD1306_Command(uint8_t command) {
    uint8_t buffer[2];
    buffer[0] = 0x00; // Command mode
    buffer[1] = command;
    
    return (I2C_TransmitData(SSD1306_I2C_ADDR, buffer, 2) == I2C_OK);
}

// Initialize SSD1306 OLED display
bool SSD1306_Init(void) {
    // Short delay for display power-up
    // Simple delay function implementation
    for (volatile uint32_t i = 0; i < 1000000; i++);
    
    // Initialize display with sequence of commands
    if (!SSD1306_Command(0xAE)) return false; // Display off
    if (!SSD1306_Command(0xD5)) return false; // Set display clock
    if (!SSD1306_Command(0x80)) return false; // Recommended value
    if (!SSD1306_Command(0xA8)) return false; // Set multiplex
    if (!SSD1306_Command(0x3F)) return false; // 1/64 duty
    if (!SSD1306_Command(0xD3)) return false; // Set display offset
    if (!SSD1306_Command(0x00)) return false; // No offset
    if (!SSD1306_Command(0x40)) return false; // Start line address
    if (!SSD1306_Command(0x8D)) return false; // Charge pump
    if (!SSD1306_Command(0x14)) return false; // Enable charge pump
    if (!SSD1306_Command(0x20)) return false; // Memory mode
    if (!SSD1306_Command(0x00)) return false; // Horizontal addressing
    if (!SSD1306_Command(0xA1)) return false; // Segment remap
    if (!SSD1306_Command(0xC8)) return false; // COM scan direction
    if (!SSD1306_Command(0xDA)) return false; // COM pins
    if (!SSD1306_Command(0x12)) return false; // COM pins configuration
    if (!SSD1306_Command(0x81)) return false; // Set contrast
    if (!SSD1306_Command(0xCF)) return false; // Contrast value
    if (!SSD1306_Command(0xD9)) return false; // Pre-charge period
    if (!SSD1306_Command(0xF1)) return false; // Pre-charge value
    if (!SSD1306_Command(0xDB)) return false; // VCOMH detect
    if (!SSD1306_Command(0x40)) return false; // VCOMH value
    if (!SSD1306_Command(0xA4)) return false; // Display from RAM
    if (!SSD1306_Command(0xA6)) return false; // Normal display (not inverted)
    if (!SSD1306_Command(0xAF)) return false; // Display on
    
    // Clear buffer and update display
    SSD1306_Clear();
    return SSD1306_Display();
}

// Clear display buffer
void SSD1306_Clear(void) {
    memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));
}

// Update display with current buffer
bool SSD1306_Display(void) {
    // Set column address range
    if (!SSD1306_Command(0x21)) return false; // Column address command
    if (!SSD1306_Command(0x00)) return false; // Start column
    if (!SSD1306_Command(0x7F)) return false; // End column
    
    // Set page address range
    if (!SSD1306_Command(0x22)) return false; // Page address command
    if (!SSD1306_Command(0x00)) return false; // Start page
    if (!SSD1306_Command(0x07)) return false; // End page
    
    // Send buffer to display (in chunks of 16 bytes)
    uint8_t data[17]; // 1 byte control + 16 bytes data
    data[0] = 0x40; // Data mode
    
    for (uint16_t i = 0; i < sizeof(SSD1306_Buffer); i += 16) {
        uint16_t chunk_size = sizeof(SSD1306_Buffer) - i;
        if (chunk_size > 16) chunk_size = 16;
        
        memcpy(&data[1], &SSD1306_Buffer[i], chunk_size);
        
        if (I2C_TransmitData(SSD1306_I2C_ADDR, data, chunk_size + 1) != I2C_OK) {
            return false;
        }
    }
    
    return true;
}

// Draw pixel at specific coordinates
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    
    // Calculate buffer position
    uint16_t index = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_position = y % 8;
    
    // Set or clear bit based on color
    if (color) {
        SSD1306_Buffer[index] |= (1 << bit_position);
    } else {
        SSD1306_Buffer[index] &= ~(1 << bit_position);
    }
}

// Draw character (implementation details omitted for brevity)
static void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch) {
    // Not fully implemented in this blueprint
}

// Draw text at specific coordinates
void SSD1306_DrawText(uint8_t x, uint8_t y, const char* text) {
    // Not fully implemented in this blueprint
}

// Draw simple graph
void SSD1306_DrawGraph(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float* data, uint8_t data_len) {
    // Not fully implemented in this blueprint
}
```

#### AHT10 Sensor Driver
```c
// aht10.h
#ifndef AHT10_H
#define AHT10_H

#include <stdint.h>
#include <stdbool.h>

// AHT10 I2C address
#define AHT10_I2C_ADDR 0x38

// Initialize AHT10 sensor
bool AHT10_Init(void);

// Read temperature and humidity from AHT10 sensor
bool AHT10_ReadData(float* temperature, float* humidity);

#endif // AHT10_H
```

```c
// aht10.c
#include "aht10.h"
#include "i2c.h"

// Commands
#define AHT10_CMD_CALIBRATE 0xE1
#define AHT10_CMD_MEASURE   0xAC
#define AHT10_CMD_RESET     0xBA

// Simple delay function (not using timers for simplicity)
static void AHT10_Delay(uint32_t ms) {
    // Simple delay function implementation
    for (volatile uint32_t i = 0; i < ms * 10000; i++);
}

// Initialize AHT10 sensor
bool AHT10_Init(void) {
    uint8_t cmd[3];
    
    // Soft reset
    cmd[0] = AHT10_CMD_RESET;
    if (I2C_TransmitData(AHT10_I2C_ADDR, cmd, 1) != I2C_OK) {
        return false;
    }
    
    // Wait for reset to complete
    AHT10_Delay(20);
    
    // Calibration command (0xE1)
    cmd[0] = AHT10_CMD_CALIBRATE;
    cmd[1] = 0x08;
    cmd[2] = 0x00;
    if (I2C_TransmitData(AHT10_I2C_ADDR, cmd, 3) != I2C_OK) {
        return false;
    }
    
    // Wait for calibration
    AHT10_Delay(10);
    
    return true;
}

// Read temperature and humidity from AHT10 sensor
bool AHT10_ReadData(float* temperature, float* humidity) {
    uint8_t cmd[3] = {AHT10_CMD_MEASURE, 0x33, 0x00};
    uint8_t data[6];
    
    // Send measurement command
    if (I2C_TransmitData(AHT10_I2C_ADDR, cmd, 3) != I2C_OK) {
        return false;
    }
    
    // Wait for measurement (~80ms)
    AHT10_Delay(80);
    
    // Read back data
    if (I2C_ReceiveData(AHT10_I2C_ADDR, data, 6) != I2C_OK) {
        return false;
    }
    
    // Check if busy flag is 0
    if (data[0] & 0x80) {
        return false; // Sensor is busy
    }
    
    // Process humidity data (20 bits)
    uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *humidity = (float)humidity_raw * 100.0f / 1048576.0f;
    
    // Process temperature data (20 bits)
    uint32_t temp_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temperature = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
    
    return true;
}
```

#### SD Card/FatFS Implementation
```c
// sd_card.h
#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>
#include <stdbool.h>

// SD card commands
#define SD_CMD0      0 // GO_IDLE_STATE
#define SD_CMD1      1 // SEND_OP_COND
#define SD_CMD8      8 // SEND_IF_COND
#define SD_CMD9      9 // SEND_CSD
#define SD_CMD10     10 // SEND_CID
#define SD_CMD12     12 // STOP_TRANSMISSION
#define SD_CMD16     16 // SET_BLOCKLEN
#define SD_CMD17     17 // READ_SINGLE_BLOCK
#define SD_CMD18     18 // READ_MULTIPLE_BLOCK
#define SD_CMD24     24 // WRITE_BLOCK
#define SD_CMD25     25 // WRITE_MULTIPLE_BLOCK
#define SD_CMD55     55 // APP_CMD
#define SD_CMD58     58 // READ_OCR
#define SD_ACMD41    41 // SD_SEND_OP_COND

// Initialize SD card
bool SD_Init(void);

// Read a single block
bool SD_ReadBlock(uint32_t block_addr, uint8_t* buffer);

// Write a single block
bool SD_WriteBlock(uint32_t block_addr, const uint8_t* buffer);

// Open or create log file
bool SD_OpenLogFile(void);

// Log data to SD card
bool SD_LogData(float temp, float humidity, uint32_t timestamp);

// Close log file
bool SD_CloseLogFile(void);

#endif // SD_CARD_H
```

```c
// sd_card.c
#include "sd_card.h"
#include "spi.h"

// Block size (fixed for SD cards)
#define SD_BLOCK_SIZE 512

// Card type
static uint8_t card_type = 0;

// Send command to SD card
static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg) {
    uint8_t response;
    uint8_t crc = 0x01; // Dummy CRC, not used except for CMD0
    
    if (cmd == SD_CMD0) crc = 0x95; // Valid CRC for CMD0
    if (cmd == SD_CMD8) crc = 0x87; // Valid CRC for CMD8
    
    // Select card
    SPI_ChipSelect(0);
    
    // Send command
    SPI_TransmitReceiveByte(0x40 | cmd);
    SPI_TransmitReceiveByte((arg >> 24) & 0xFF);
    SPI_TransmitReceiveByte((arg >> 16) & 0xFF);
    SPI_TransmitReceiveByte((arg >> 8) & 0xFF);
    SPI_TransmitReceiveByte(arg & 0xFF);
    SPI_TransmitReceiveByte(crc);
    
    // Wait for response (timeout after 10 attempts)
    uint8_t timeout = 10;
    do {
        response = SPI_TransmitReceiveByte(0xFF);
        timeout--;
    } while ((response & 0x80) && timeout);
    
    return response;
}

// Initialize SD card
bool SD_Init(void) {
    uint8_t r1;
    uint16_t count;
    
    // Set DI and CS high
    SPI_ChipSelect(1);
    
    // Send at least 74 clock pulses
    for (uint8_t i = 0; i < 10; i++) {
        SPI_TransmitReceiveByte(0xFF);
    }
    
    // Enter idle state
    count = 100;
    do {
        r1 = SD_SendCommand(SD_CMD0, 0);
        count--;
    } while (r1 != 0x01 && count);
    
    if (r1 != 0x01) {
        SPI_ChipSelect(1);
        return false; // Failed to enter idle state
    }
    
    // Send interface conditions (CMD8)
    r1 = SD_SendCommand(SD_CMD8, 0x000001AA);
    
    if (r1 & 0x04) {
        // Version 1 SD Card
        card_type = 1;
        
        // Initialize card
        count = 1000;
        do {
            // ACMD41 is preceded by CMD55
            SD_SendCommand(SD_CMD55, 0);
            r1 = SD_SendCommand(SD_ACMD41, 0);
            count--;
        } while (r1 && count);
    } else {
        // Version 2 SD Card
        card_type = 2;
        
        // Read 4 bytes
        SPI_TransmitReceiveByte(0xFF);
        SPI_TransmitReceiveByte(0xFF);
        SPI_TransmitReceiveByte(0xFF);
        SPI_TransmitReceiveByte(0xFF);
        
        // Initialize card
        count = 1000;
        do {
            // ACMD41 is preceded by CMD55
            SD_SendCommand(SD_CMD55, 0);
            r1 = SD_SendCommand(SD_ACMD41, 0x40000000); // HCS bit set
            count--;
        } while (r1 && count);
    }
    
    if (r1) {
        SPI_ChipSelect(1);
        return false; // Failed to initialize
    }
    
    // Set block size to 512 bytes
    r1 = SD_SendCommand(SD_CMD16, SD_BLOCK_SIZE);
    
    SPI_ChipSelect(1);
    return (r1 == 0);
}

// Read a single block
bool SD_ReadBlock(uint32_t block_addr