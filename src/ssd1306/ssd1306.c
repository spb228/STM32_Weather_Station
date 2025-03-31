#include "ssd1306.h"
#include "uart.h"

// Buffer for the display (128x64 pixels = 1024 bytes)
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

void SSD1306_Init(void) {
    // Initialize I2C1
    I2C1_Init();
    
    // Wait for the display to power up
    delay_ms(100);
    
    // Send initialization commands
    SSD1306_WriteCommand(SSD1306_DISPLAYOFF);             // 0xAE
    SSD1306_WriteCommand(SSD1306_SETDISPLAYCLOCKDIV);     // 0xD5
    SSD1306_WriteCommand(0x80);                           // Default value
    SSD1306_WriteCommand(SSD1306_SETMULTIPLEX);           // 0xA8
    SSD1306_WriteCommand(0x3F);                           // 1/64 duty
    SSD1306_WriteCommand(SSD1306_SETDISPLAYOFFSET);       // 0xD3
    SSD1306_WriteCommand(0x00);                           // No offset
    SSD1306_WriteCommand(SSD1306_SETSTARTLINE | 0x00);    // Line #0
    SSD1306_WriteCommand(SSD1306_CHARGEPUMP);             // 0x8D
    SSD1306_WriteCommand(0x14);                           // Enable charge pump
    SSD1306_WriteCommand(SSD1306_MEMORYMODE);             // 0x20
    SSD1306_WriteCommand(0x00);                           // Horizontal addressing mode
    SSD1306_WriteCommand(SSD1306_SEGREMAP | 0x01);        // Flip horizontally
    SSD1306_WriteCommand(SSD1306_COMSCANDEC);             // Flip vertically
    SSD1306_WriteCommand(SSD1306_SETCOMPINS);             // 0xDA
    SSD1306_WriteCommand(0x12);                           // Alternative COM pin config
    SSD1306_WriteCommand(SSD1306_SETCONTRAST);            // 0x81
    SSD1306_WriteCommand(0xCF);                           // Max contrast
    SSD1306_WriteCommand(SSD1306_SETPRECHARGE);           // 0xD9
    SSD1306_WriteCommand(0xF1);                           // Pre-charge period
    SSD1306_WriteCommand(SSD1306_SETVCOMDETECT);          // 0xDB
    SSD1306_WriteCommand(0x40);                           // VCOMH deselect level
    SSD1306_WriteCommand(SSD1306_DISPLAYALLON_RESUME);    // 0xA4
    SSD1306_WriteCommand(SSD1306_NORMALDISPLAY);          // 0xA6
    
    // Clear the display
    SSD1306_Clear();
    SSD1306_Display();
    
    // Turn on the display
    SSD1306_WriteCommand(SSD1306_DISPLAYON);              // 0xAF
}

void SSD1306_Clear(void) {
    // Clear the buffer
    for (uint16_t i = 0; i < (SSD1306_WIDTH * SSD1306_HEIGHT / 8); i++) {
        SSD1306_Buffer[i] = 0x00;
    }
}

void SSD1306_Display(void) {
    // Set column address range
    SSD1306_WriteCommand(SSD1306_SETCOLADDR);
    SSD1306_WriteCommand(0x00);                          // Start column
    SSD1306_WriteCommand(SSD1306_WIDTH - 1);             // End column
    
    // Set page address range
    SSD1306_WriteCommand(SSD1306_SETPAGEADDR);
    SSD1306_WriteCommand(0x00);                          // Start page
    SSD1306_WriteCommand(0x07);                          // End page
    
    // Send display buffer
    SSD1306_WriteData(SSD1306_Buffer, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y) {
    // Check if coordinates are valid
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    // Calculate the byte position in the buffer
    uint16_t byte_pos = x + (y / 8) * SSD1306_WIDTH;
    // Calculate the bit position in the byte
    uint8_t bit_pos = y % 8;
    
    // Set the pixel
    SSD1306_Buffer[byte_pos] |= (1 << bit_pos);
}

void SSD1306_WriteCommand(uint8_t command) {
    I2C1_Start();
    I2C1_SendAddr(SSD1306_I2C_ADDR << 1);
    I2C1_SendByte(SSD1306_COMMAND);
    I2C1_SendByte(command);
    I2C1_Stop();
}

void SSD1306_WriteData(uint8_t* data, uint16_t size) {
    I2C1_Start();
    I2C1_SendAddr(SSD1306_I2C_ADDR << 1);
    I2C1_SendByte(SSD1306_DATA);
    
    for (uint16_t i = 0; i < size; i++) {
        I2C1_SendByte(data[i]);
    }
    
    I2C1_Stop();
}

void I2C1_Init(void) {
    // Enable GPIOB clock
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Enable I2C1 clock
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // Configure PB8 (SCL) and PB9 (SDA) for alternate function 4
    
    // Set mode to alternate function
    GPIOB_MODER &= ~(3U << (8 * 2));
    GPIOB_MODER |= (GPIO_MODER_AF << (8 * 2));
    GPIOB_MODER &= ~(3U << (9 * 2));
    GPIOB_MODER |= (GPIO_MODER_AF << (9 * 2));
    
    // Set output type to open-drain
    GPIOB_OTYPER |= (GPIO_OTYPER_OD << 8);
    GPIOB_OTYPER |= (GPIO_OTYPER_OD << 9);
    
    // Set speed to high
    GPIOB_OSPEEDR &= ~(3U << (8 * 2));
    GPIOB_OSPEEDR |= (GPIO_OSPEEDR_HIGH << (8 * 2));
    GPIOB_OSPEEDR &= ~(3U << (9 * 2));
    GPIOB_OSPEEDR |= (GPIO_OSPEEDR_HIGH << (9 * 2));
    
    // Set pull-up
    GPIOB_PUPDR &= ~(3U << (8 * 2));
    GPIOB_PUPDR |= (GPIO_PUPDR_PU << (8 * 2));
    GPIOB_PUPDR &= ~(3U << (9 * 2));
    GPIOB_PUPDR |= (GPIO_PUPDR_PU << (9 * 2));
    
    // Set alternate function 4 (I2C1)
    // PB8 and PB9 are in the high register (AFRH)
    GPIOB_AFRH &= ~(0xF << ((8 - 8) * 4));
    GPIOB_AFRH |= (GPIO_AF4 << ((8 - 8) * 4));
    GPIOB_AFRH &= ~(0xF << ((9 - 8) * 4));
    GPIOB_AFRH |= (GPIO_AF4 << ((9 - 8) * 4));
    
    // Reset I2C1
    RCC_APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC_APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    
    // Set I2C clock frequency (16 MHz)
    I2C1_CR2 = I2C_CR2_FREQ_42MHZ;
    
    // Configure I2C for 100 kHz standard mode
    I2C1_CCR = 210;  // 42MHz / (2 * 100kHz)
    
    // Configure rise time
    I2C1_TRISE = 43;  // (1000ns / (1/42MHz)) + 1
    
    // Enable I2C1
    I2C1_CR1 |= I2C_CR1_PE;
}

void I2C1_Start(void) {
    // Send start condition
    I2C1_CR1 |= I2C_CR1_START;
    
    // Wait for start bit to be set
    while (!(I2C1_SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void) {
    // Send stop condition
    I2C1_CR1 |= I2C_CR1_STOP;
    
    // Wait a bit for the stop condition to complete
    delay_ms(1);
}

void I2C1_SendAddr(uint8_t addr) {
    // Send slave address
    I2C1_DR = addr;
    
    // Wait for address bit to be set
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    
    // Clear address bit by reading SR2
    (void) I2C1_SR2;
}

void I2C1_SendByte(uint8_t data) {
    // Wait for transmit buffer to be empty
    while (!(I2C1_SR1 & I2C_SR1_TXE));
    
    // Send data
    I2C1_DR = data;
    
    // Wait for byte transfer to complete
    while (!(I2C1_SR1 & I2C_SR1_BTF));
}

void delay_ms(uint32_t ms) {
    // Simple delay function (very approximate)
    // Assuming 16MHz clock
    ms *= 4000;  // Adjust this value for your specific clock
    while (ms--) {
        __asm("NOP");
    }
}