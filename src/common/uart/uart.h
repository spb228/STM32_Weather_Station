#include <string.h>
#include "common/common_def.h"

#define USART2_CTS_PIN      0
#define USART2_RTS_PIN      1
#define USART2_TX_PIN       2
#define USART2_RX_PIN       3
#define USART2_CK_PIN       4

#define UART_SUCCESS 0
#define UART_NULL_PTR 1
#define UART_EMPTY_STR 2

void usart2_gpio_init(void);

void usart2_config(void);

void usart2_send_char(const char);

uint8_t usart2_send_str(char *);