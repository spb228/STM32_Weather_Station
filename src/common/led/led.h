#ifndef LED_H
#define LED_H

#include "common/common_def.h"

#define LED_PIN     5 // PA5 pin

void init_led(void);

void led_delay(uint32_t);

void toggle_led(void);

#endif // LED_H