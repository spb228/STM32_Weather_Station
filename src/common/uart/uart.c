#include "uart.h"

/*

Using USART2 since that is routed to ST-Link Debugger.
USART2 is on APB1

*/

void usart2_gpio_init(void)
{
    GPIOA_MODER |= (2 << (USART2_CTS_PIN * 2)) |    //  Alt Func Mode PA0 -> USART2 CTS
                   (2 << (USART2_RTS_PIN * 2)) |    //  Alt Func Mode PA1 -> USART2 RTS
                   (2 << (USART2_TX_PIN * 2))  |    //  Alt Func Mode PA2 -> USART2 TX
                   (2 << (USART2_RX_PIN * 2))  |    //  Alt Func Mode PA3 -> USART2 RX
                   (2 << (USART2_CK_PIN * 2));      //  Alt Func Mode PA4 -> USART2 CK

    GPIOA_OSPEEDR |= //(2 << (USART2_CTS_PIN * 2)) | //  High Speed Mode for all UART pins
                     (2 << (USART2_RTS_PIN * 2)) |    
                     (2 << (USART2_TX_PIN * 2))  |   
                     //(2 << (USART2_RX_PIN * 2))  |    
                     (2 << (USART2_CK_PIN * 2));

    GPIOA_PUPDR |= (1 << (USART2_TX_PIN * 2)) |       // pull up resistor
                   (1 << (USART2_RX_PIN * 2));        // pull up resistor

    GPIOA_AFRL |= (7 << (USART2_CTS_PIN * 4)) |    //  Alt Func 7
                  (7 << (USART2_RTS_PIN * 4)) |    //  Alt Func 7
                  (7 << (USART2_TX_PIN * 4))  |    //  Alt Func 7
                  (7 << (USART2_RX_PIN * 4))  |    //  Alt Func 7
                  (7 << (USART2_CK_PIN * 4));      //  Alt Func 7

    RCC_AHB1ENR |= (1 << 0);    // enable GPIOA clock
    RCC_APB1ENR |= (1 << 17);   // Enable RCC for USART2
}

void usart2_config()
{
    usart2_gpio_init();
    
    USART2_CR1 &= ~(1 << 13); // disable USART2

    /*
    baud rate calc (115200):
        fclk = 21 Mhz (APB1 clock speed)
        usartdiv = fclk / (16 * baud) = 11.39
        usart_div_mantissa = 11
        usart_div_fraction = 0.39 * 16 = 6.24 = 6
    */
    USART2_BRR = 0; // clear
    USART2_BRR = (6 << 0) |     // fraction for 115200 baud rate
                 (11 << 4);    // mantissa for 115200 baud rate

    //USART2_CR1 &= ~(1 << 12);  // 8 data bits for stlink debugger. 0 by default. 

    USART2_CR1 |= (1 << 10) |  // Parity control enabled
                  (1 << 9);     // even parity

    //USART2_CR2 &= ~(1 << 12);    // 1 stop bit is typical for uart. 0 by default. 

    //USART2_CR3 &= ~(1 << 8);    // RTS disabled - not supported by stlink. 0 by default. 
    //USART2_CR3 &= ~(1 << 9);    // CTS disabled - not supported by stlink. 0 by default. 

    USART2_CR1 |= (1 << 2) |     // receiver enabled
                  (1 << 3);      // transmitter enabled

    USART2_CR1 |= (1 << 13);     // enable USART2
}

// void usart2_config(void)
// {
//     // Enable clocks
//     RCC_AHB1ENR |= (1 << 0);   // GPIOA
//     RCC_APB1ENR |= (1 << 17);  // USART2
    
//     // Configure TX pin only (PA2)
//     GPIOA_MODER &= ~(3 << (2*2));  // Clear bits for PA2
//     GPIOA_MODER |= (2 << (2*2));   // Set alternate function mode
    
//     GPIOA_AFRL &= ~(0xF << (2*4)); // Clear AF bits for PA2
//     GPIOA_AFRL |= (7 << (2*4));    // Set AF7 for USART2
    
//     // Try 9600 baud for easier debugging
//     USART2_BRR = (12 << 0) | (136 << 4);  // 9600 baud with 21MHz
    
//     // Basic USART2 config
//     USART2_CR1 = 0;           // Clear all settings
//     USART2_CR1 |= (1 << 3);   // Enable transmitter
//     USART2_CR1 |= (1 << 13);  // Enable USART
// }

void usart2_send_char(const char c)
{
    // wait until transmit data reg is empty
    while (!(USART2_SR & (1 << 7))); // polling TXE instead of interrupt
    
    USART2_DR = c; 

    // wait until transmit is complete
    while (!(USART2_SR & (1 << 6)));

    // TODO: handle newline conversion (sending CR+LF when \n is requested)
}