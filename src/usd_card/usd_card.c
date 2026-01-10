#include "usd_card.h"

#define NSS     4
#define SCK     5
#define MISO    6
#define MOSI    7

static void SPI1_GPIO_Init(void)
{
    // Enable GPIOA and SPI1 clocks
    RCC_AHB1ENR |= (1 << 0);      // GPIOA clock
    RCC_APB2ENR |= (1 << 12);     // SPI1 clock

    // Set MODER
    GPIOA_MODER &= ~((3 << (NSS*2)) | (3 << (SCK*2)) | (3 << (MISO*2)) | (3 << (MOSI*2)));
    GPIOA_MODER |=  (2 << (NSS*2)) | (2 << (SCK*2)) | (2 << (MISO*2)) | (2 << (MOSI*2));

    // Push-pull
    GPIOA_OTYPER &= ~((1 << NSS) | (1 << SCK) | (1 << MOSI));

    // Speed: high
    GPIOA_OSPEEDR |= (2 << (NSS*2)) | (2 << (SCK*2)) | (2 << (MISO*2)) | (2 << (MOSI*2));

    // Pull-up on MISO
    GPIOA_PUPDR &= ~(3 << (MISO*2));
    GPIOA_PUPDR |=  (1 << (MISO*2));

    // AFRL -> AF5 (SPI1)
    GPIOA_AFRL &= ~((0xF << (NSS*4)) | (0xF << (SCK*4)) | (0xF << (MISO*4)) | (0xF << (MOSI*4)));
    GPIOA_AFRL |=  (5 << (NSS*4)) | (5 << (SCK*4)) | (5 << (MISO*4)) | (5 << (MOSI*4));
}

void SPI1_Init(void)
{
    SPI1_GPIO_Init();

    SPI1_CR1 = 0;
    SPI1_CR1 |= (1 << 2);     // Master mode
    SPI1_CR1 |= (3 << 3);     // Baud rate = fPCLK/16
    SPI1_CR1 |= (1 << 9);     // Software slave management disabled (hardware NSS)
    SPI1_CR1 |= (1 << 1);     // CPOL = 0
    SPI1_CR1 |= (1 << 0);     // CPHA = 0
    SPI1_CR1 |= (0b111 << 3); // BR[2:0]: Baud rate control /256 (slowest)

    SPI1_CR2 = 0;
    SPI1_CR2 |= (1 << 2); // SSOE: SS output enable

    SPI1_CR1 |= (1 << 6);     // SPI enable
}

// Low-level SPI transmit and receive
static void SPI1_Transmit(uint8_t data)
{
    while (!(SPI1_SR & (1 << 1))); // Wait until TXE
    SPI1_DR = data;
    while (!(SPI1_SR & (1 << 0))); // Wait until RXNE
    (void)SPI1_DR;
}

static uint8_t SPI1_Transfer(uint8_t data)
{
    while (!(SPI1_SR & (1 << 1)));
    SPI1_DR = data;
    while (!(SPI1_SR & (1 << 0)));
    return SPI1_DR;
}

void SD_SendDummyClocks(void)
{
    GPIOA_ODR |= (1 << 4); // CS high
    for (int i = 0; i < 10; i++)
        SPI1_Transmit(0xFF);
}

uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t response = 0xFF;

    GPIOA_ODR &= ~(1 << 4); // CS low

    SPI1_Transmit(0x40 | cmd);
    SPI1_Transmit((arg >> 24) & 0xFF);
    SPI1_Transmit((arg >> 16) & 0xFF);
    SPI1_Transmit((arg >> 8) & 0xFF);
    SPI1_Transmit(arg & 0xFF);
    SPI1_Transmit(crc);

    for (int i = 0; i < 8; i++) {
        response = SPI1_Transfer(0xFF);
        if (response != 0xFF)
            break;
    }

    GPIOA_ODR |= (1 << 4); // CS high
    SPI1_Transmit(0xFF);   // One extra clock

    return response;
}

void SD_Init(void)
{
    SD_SendDummyClocks();

    for (int i = 0; i < 5; i++) {
        uint8_t r = SD_SendCommand(0, 0, 0x95); // CMD0
        if (r == 0x01) {
            break;
        }
    }
}

uint8_t SPI1_Receive(void)
{
    // Send dummy byte to generate clock
    SPI1_Transmit(0xFF);

    // Wait for received byte
    while (!(SPI1_SR & (1 << 0))); // Wait until RXNE is set

    return (uint8_t)SPI1_DR;
}

uint8_t SD_Send_CMD8(void)
{
    SD_SendCommand(0x48, 0x000001AA, 0x87); // CMD8 with pattern and CRC
    
    uint8_t r1;
    for (int i = 0; i < 8; i++)
    {
        r1 = SPI1_Receive();
        if ((r1 & 0x80) == 0)
            break;
    }

    if (r1 != 0x01)
    {
        print("CMD8 failed: R1 != 0x01\r\n");
        return 1;
    }

    // Read the rest of the R7 response (4 bytes)
    uint8_t response[4];
    for (int i = 0; i < 4; i++)
    {
        response[i] = SPI1_Receive();
    }

    if (response[2] == 0x01 && response[3] == 0xAA)
    {
        print("CMD8 success: Card is SDv2 and voltage range OK\r\n");
        return 0;
    }
    else
    {
        print("CMD8 error: Pattern mismatch\r\n");
        return 2;
    }
}