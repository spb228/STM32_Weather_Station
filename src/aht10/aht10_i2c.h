#include "common/common_def.h"

#define I2C1_SCL_PIN        8
#define I2C1_SDA_PIN        9

#define I2C1_CR1_START      (1 << 8);
#define I2C1_CR1_STOP       (1 << 9); 
#define I2C1_CR1_ACK        (1 << 10);

#define I2C1_EV_INTERRUPT_NUM   31
#define I2C1_ER_INTERRUPT_NUM   32

void i2c1_gpio_init(void);

void i2c1_config(void);

void I2C1_StartTransaction(uint8_t, uint8_t *,uint16_t, uint8_t *, uint16_t);

void I2C1_EV_IRQHandler(void);

void I2C1_ER_IRQHandler(void);
