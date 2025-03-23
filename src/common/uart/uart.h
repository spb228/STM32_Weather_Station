#ifndef UART_H
#define UART_H

#include <string.h>
#include "common/common_def.h"
#include "common/systick/systick.h"

#define USART2_CTS_PIN          0
#define USART2_RTS_PIN          1
#define USART2_TX_PIN           2
#define USART2_RX_PIN           3
#define USART2_CK_PIN           4

#define USART2_SUCCESS          0
#define USART2_NULL_PTR         1
#define USART2_EMPTY_STR        2
#define USART2_TX_DISABLED      3
#define USART2_TX_TIMEOUT       4
#define USART2_BUFFER_FULL      5
#define USART2_TX_BUSY          6

#define USART2_CHAR_TIMEOUT     20

#define USART2_INTERRUPT_NUM    38

#define USART2_CR1_TCIE         (1 << 6)
#define USART2_CR1_TXEIE        (1 << 7)
#define USART2_SR_TC            (1 << 6)
#define USART2_SR_TXE           (1 << 7)

#define USART2_TX_BUFFER_SIZE     128

void usart2_gpio_init(void);

void usart2_config(void);

uint8_t print(char *);

#endif // UART_H