#ifndef SYSTICK_H
#define SYSTICK_H

#include "common/common_def.h"

void systick_init(void);

void SysTick_Handler(void);

uint32_t get_tick(void);

uint8_t is_timeout_elapsed(uint32_t, uint32_t);

#endif // SYSTICK_H