#include "uart.h"

/*

Using USART2 since that is routed to ST-Link Debugger.
USART2 is on APB1

*/

void usart2_gpio_init(void)
{
    RCC_AHB1ENR |= (1 << 0);    // enable GPIOA clock
    RCC_APB1ENR |= (1 << 17);   // Enable RCC for USART2

    GPIOA_MODER |= (2 << (USART2_CTS_PIN * 2)) |    //  Alt Func Mode PA0 -> USART2 CTS
                   (2 << (USART2_RTS_PIN * 2)) |    //  Alt Func Mode PA1 -> USART2 RTS
                   (2 << (USART2_TX_PIN * 2))  |    //  Alt Func Mode PA2 -> USART2 TX
                   (2 << (USART2_RX_PIN * 2))  |    //  Alt Func Mode PA3 -> USART2 RX
                   (2 << (USART2_CK_PIN * 2));      //  Alt Func Mode PA4 -> USART2 CK

    GPIOA_OSPEEDR |= (2 << (USART2_CTS_PIN * 2)) | //  High Speed Mode for all UART pins
                     (2 << (USART2_RTS_PIN * 2)) |    
                     (2 << (USART2_TX_PIN * 2))  |   
                     (2 << (USART2_RX_PIN * 2))  |    
                     (2 << (USART2_CK_PIN * 2));

    GPIOA_PUPDR |= (1 << (USART2_TX_PIN * 2)) |    // pull up resistor
                   (1 << (USART2_RX_PIN * 2));     // pull up resistor

    GPIOA_AFRL |= (7 << (USART2_CTS_PIN * 4)) |    //  Alt Func 7
                  (7 << (USART2_RTS_PIN * 4)) |    //  Alt Func 7
                  (7 << (USART2_TX_PIN * 4))  |    //  Alt Func 7
                  (7 << (USART2_RX_PIN * 4))  |    //  Alt Func 7
                  (7 << (USART2_CK_PIN * 4));      //  Alt Func 7
}

void usart2_config()
{
    usart2_gpio_init();
    
    USART2_CR1 = 0;
    USART2_CR1 &= ~(1 << 13);   // disable USART2
    USART2_CR1 &= ~(1 << 12);   // 8 data bits for stlink debugger. 0 by default. 
    //USART2_CR1 |= (1 << 10) |   // Parity control enabled
    //              (1 << 9);     // even parity

    USART2_CR2 = 0; 
    USART2_CR2 &= ~(1 << 12);   // 1 stop bit is typical for uart. 0 by default. 

    USART2_CR3 = 0; 
    USART2_CR3 &= ~(1 << 8);   // RTS disabled - not supported by stlink. 0 by default. 
    USART2_CR3 &= ~(1 << 9);   // CTS disabled - not supported by stlink. 0 by default. 

    /*
    baud rate calc (115200):
        fclk = 42 Mhz (APB1 clock speed)
        usartdiv = fclk / (16 * baud) = 22.79
        usart_div_mantissa = 22
        usart_div_fraction = 0.79 * 16 = 12.64 = 13
    */
    USART2_BRR = 0;
    USART2_BRR = (13 << 0) |     // fraction for 115200 baud rate
                 (22 << 4);      // mantissa for 115200 baud rate

    USART2_CR1 |= (1 << 2) |     // receiver enabled
                  (1 << 3);      // transmitter enabled

    USART2_CR1 |= (1 << 13);     // enable USART2
}

uint8_t usart2_send_char(const char c)
{
    // if usart2 is disabled, return error
    if (!(USART2_CR1 & (1 << 13)))
    {
        return USART2_TX_DISABLED;
    }

    // wait until transmit data reg is empty
    volatile uint32_t start_tick_usart2_sr = get_tick(); 
    while (!(USART2_SR & (1 << 7)))
    {
        if (is_timeout_elapsed(start_tick_usart2_sr, USART2_CHAR_TIMEOUT))
        {
            return USART2_TX_TIMEOUT;
        }
    }

    if (c == '\n')
    {
        USART2_DR = '\r'; // send carriage return

        // wait until transmission complete
        start_tick_usart2_sr = get_tick();
        while (!(USART2_SR & (1 << 6))) 
        {
            if (is_timeout_elapsed(start_tick_usart2_sr, USART2_CHAR_TIMEOUT))
            {
                return USART2_TX_TIMEOUT;
            }
        } 

        // wait until transmission data is empty
        start_tick_usart2_sr = get_tick();
        while (!(USART2_SR & (1 << 7))) 
        {
            if (is_timeout_elapsed(start_tick_usart2_sr, USART2_CHAR_TIMEOUT))
            {
                return USART2_TX_TIMEOUT;
            }
        }

        USART2_DR = '\n'; // send next line
    }
    else 
    {
        USART2_DR = c; 
    }
    
    // wait until transmit is complete
    start_tick_usart2_sr = get_tick();
    while (!(USART2_SR & (1 << 6)))
    {
        if (is_timeout_elapsed(start_tick_usart2_sr, USART2_CHAR_TIMEOUT))
        {
            return USART2_TX_TIMEOUT;
        }
    }

    return USART2_SUCCESS;
}

uint8_t usart2_send_str(char *c)
{
    // check if pointer is null or string is empty
    if (c == NULL)
    {
        return USART2_NULL_PTR;  
    }
    else if (*c == '\0')
    {
        return USART2_EMPTY_STR;
    }

    while (*c != '\0')
    {
        uint8_t status = usart2_send_char(*c);
        if (status != USART2_SUCCESS)
        {
            return status;
        } 
        c++; 
    }

    return USART2_SUCCESS;
}