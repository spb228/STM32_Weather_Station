#include "uart.h"

/*

Using USART2 since that is routed to ST-Link Debugger.
USART2 is on APB1

*/

// static uint8_t usart2_tx_buffer[UART_TX_BUFFER_SIZE];
volatile char *tx_buffer;
volatile uint32_t tx_index;
volatile uint32_t tx_data_len;

void usart2_gpio_init(void)
{
    RCC_AHB1ENR |= (1 << 0);  // enable GPIOA clock
    RCC_APB1ENR |= (1 << 17); // Enable RCC for USART2

    GPIOA_MODER |= (2 << (USART2_TX_PIN * 2)) | //  Alt Func Mode PA2 -> USART2 TX
                   (2 << (USART2_RX_PIN * 2));  //  Alt Func Mode PA3 -> USART2 RX

    GPIOA_OSPEEDR |= (2 << (USART2_TX_PIN * 2)) | // high speed GPIO
                     (2 << (USART2_RX_PIN * 2));  // high speed GPIO

    GPIOA_PUPDR |= (1 << (USART2_TX_PIN * 2)) | // pull up resistor
                   (1 << (USART2_RX_PIN * 2));  // pull up resistor

    GPIOA_AFRL |= (7 << (USART2_TX_PIN * 4)) | //  Alt Func 7
                  (7 << (USART2_RX_PIN * 4));  //  Alt Func 7
}

void usart2_config()
{
    usart2_gpio_init();

    USART2_CR1 = 0;
    USART2_CR2 = 0;
    USART2_CR3 = 0;
    USART2_BRR = 0;

    /*
    baud rate calc (115200):
        fclk = 42 Mhz (APB1 clock speed)
        usartdiv = fclk / (16 * baud) = 22.79
        usart_div_mantissa = 22
        usart_div_fraction = 0.79 * 16 = 12.64 = 13
    */
    USART2_BRR = (13 << 0) | // fraction for 115200 baud rate
                 (22 << 4);  // mantissa for 115200 baud rate

    USART2_CR1 |= (1 << 2) | // receiver enabled
                  (1 << 3);  // transmitter enabled

    USART2_CR1 |= (1 << 13); // enable USART2

    NVIC_ISER1 |= (1 << (USART2_INTERRUPT_NUM - 32)); // enable USART2 interrupt in NVIC reg

    USART2_CR1 |= USART2_CR1_TXEIE; // enable trans. interrupt
}

uint8_t print(char *str)
{
    if (str == NULL)
    {
        return USART2_NULL_PTR; 
    }

    if (str[0] == '\0')
    {
        return USART2_EMPTY_STR; 
    }

    tx_buffer = str;
    tx_index = 0;
    tx_data_len = 0;
    while (tx_buffer[tx_data_len] != '\0')
    {
        tx_data_len++;
    }

    USART2_CR1 |= USART2_CR1_TXEIE;

    return USART2_SUCCESS; 
}

void USART2_IRQHandler(void)
{
    // if tx data reg empty
    if ((USART2_SR & USART2_SR_TXE) && (USART2_CR1 & USART2_CR1_TXEIE))
    {
        if (tx_index < tx_data_len)
        {
            USART2_DR = tx_buffer[tx_index++];
        }
        else
        {
            USART2_CR1 &= ~(1 << 7); // disable TXE interrupt when done
        }
    }
}