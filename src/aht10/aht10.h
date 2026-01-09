#pragma once

#include "common/common_def.h"

// RCC Enable Bits
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)       // Port B clock enabled
#define RCC_APB1ENR_I2C2EN      (1U << 22)      // I2C2 clock enabled
#define RCC_APB1RSTR_I2C2RST    (1U << 22)      // Reset I2C2

