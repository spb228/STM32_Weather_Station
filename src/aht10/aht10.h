#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "common/common_def.h"

// RCC Enable Bits
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)       // Port B clock enabled
#define RCC_APB1ENR_I2C2EN      (1U << 22)      // I2C2 clock enabled
#define RCC_APB1RSTR_I2C2RST    (1U << 22)      // Reset I2C2

// Pin Definitions
#define I2C2_SCL_PIN  10 // PB10
#define I2C2_SDA_PIN  3  // PB3

// AHT10 Address
#define AHT10_ADDR_DEFAULT  (0x38 << 1)

// Forward declarations of structs to be used in callbacks
typedef struct aht10_s aht10_t; 

// I2C2 CB struct
typedef struct
{
    void (*delay)(uint32_t ms);
    void (*init)(void);
    void (*start)(void);
    void (*stop)(void);
    void (*write)(uint8_t addr, uint8_t *data, uint16_t len);
    void (*read)(uint8_t addr, uint8_t *data, uint16_t len);
} i2c2_cb_t;

// AHT10 CB struct
typedef struct
{
    void (*reset)(aht10_t *dev); 
    void (*trigger)(aht10_t *dev);
    void (*read_raw)(aht10_t *dev);
    void (*parse)(aht10_t *dev);
} aht10_cb_t;

// AHT10 main object
struct aht10_s
{
    // hardware data
    uint8_t i2c_addr; 

    // interfaces
    i2c2_cb_t i2c; 
    aht10_cb_t ops;

    // data buffers
    uint8_t raw_data[6];

    // public results
    float temp_c; 
    float humid_pct; 
};

// Public API
void aht10_init(aht10_t *dev);
void aht10_run_cycle(aht10_t *dev); // helper to run trigger -> wait -> read -> parse
