#include "uart.h"

/*

Using USART2 since that is routed to ST-Link Debugger.
USART2 is on APB1

*/

static volatile uint8_t usart2_tx_rb[USART2_TX_BUFFER_SIZE]; // ring buffer definition
static volatile uint32_t usart2_tx_head = 0; 
static volatile uint32_t usart2_tx_tail = 0; 
static volatile uint8_t tx_busy = 0; // 0 denotes free, 1 denotes busy

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
     * baud rate calc (115200):
     * fclk = 42 Mhz (APB1 clock speed)
     * usartdiv = fclk / (16 * baud) = 22.79
     * usart_div_mantissa = 22
     * usart_div_fraction = 0.79 * 16 = 12.64 = 13
    */
    USART2_BRR = (13 << 0) | // fraction for 115200 baud rate
                 (22 << 4);  // mantissa for 115200 baud rate

    USART2_CR1 |= (1 << 2) | // receiver enabled
                  (1 << 3);  // transmitter enabled

    USART2_CR1 |= (1 << 13); // enable USART2

    NVIC_ISER1 |= (1 << (USART2_INTERRUPT_NUM - 32)); // enable USART2 interrupt in NVIC reg

    USART2_CR1 |= USART2_CR1_TXEIE; // enable trans. interrupt
}

uint8_t usart2_begin_transmission(void)
{
    if (tx_busy)
    {
        return USART2_TX_BUSY;
    }

    if (usart2_tx_head != usart2_tx_tail)
    {
        tx_busy = 1; // enable tx_busy
        USART2_CR1 |= USART2_CR1_TXEIE; // enable txe interrupt
    }

    return USART2_SUCCESS; 
}

uint8_t usart2_send_char(char c)
{
    uint32_t next_head = ((usart2_tx_head + 1) % USART2_TX_BUFFER_SIZE);

    /* TODO: possibly add a delay here to wait till buffer empty */
    /* if buffer full return error */
    if (next_head == usart2_tx_tail)
    {
        return USART2_BUFFER_FULL; 
    }

    usart2_tx_rb[usart2_tx_head] = c; 

    usart2_tx_head = next_head; 

    usart2_begin_transmission(); 

    return USART2_SUCCESS;
}

uint8_t usart2_send_str(char *str)
{
    /* while input string is not empty*/
    while (*str)
    {
        usart2_send_char(*str++); 
    }

    return USART2_SUCCESS;
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

    usart2_send_str(str); 

    return USART2_SUCCESS; 
}

void USART2_IRQHandler(void)
{
    /* if tx data reg is empty */
    if ((USART2_SR & USART2_SR_TXE) && (USART2_CR1 & USART2_CR1_TXEIE))
    {
        if (usart2_tx_tail != usart2_tx_head)
        {
            USART2_DR = usart2_tx_rb[usart2_tx_tail];
            usart2_tx_tail = ((usart2_tx_tail + 1) % USART2_TX_BUFFER_SIZE);
        }
        else
        {
            USART2_CR1 &= ~USART2_CR1_TXEIE;
            tx_busy = 0;
        }
    }

    // TODO: add TCIE functionality to check if tranmission is complete
}