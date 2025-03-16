#include "common/common_def.h"

#define USART2_CTS_PIN      0
#define USART2_RTS_PIN      1
#define USART2_TX_PIN       2
#define USART2_RX_PIN       3
#define USART2_CK_PIN       4

void usart2_gpio_init(void);

void usart2_config(void);

void usart2_send_char(const char c);