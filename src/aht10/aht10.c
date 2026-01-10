#include "aht10.h"

// --- HW delay ---
static void delay_ms(uint32_t ms) 
{
    // Tuned for 84 MHz System Clock
    // Approx 4 cycles per loop iteration -> 84,000,000 / 4 = 21,000,000 loops/sec
    // 21,000 loops = 1 ms
    ms *= 21000;  // Adjust this value for your specific clock
    while (ms--) {
        __asm("nop");
    }
}

/**********************  I2C Functions ************************/
// --- Init I2C2 ---
static void i2c2_init(void) 
{
    // 1. Enable Clocks
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_I2C2EN;

    // 2. Configure PB10 (SCL) -> AF04
    GPIOB_MODER   &= ~(3U << (I2C2_SCL_PIN * 2));
    GPIOB_MODER   |= (2U << (I2C2_SCL_PIN * 2)); // AF Mode
    GPIOB_OTYPER  |= (1U << I2C2_SCL_PIN);       // Open Drain
    GPIOB_OSPEEDR |= (2U << (I2C2_SCL_PIN * 2)); // High Speed
    GPIOB_PUPDR   &= ~(3U << (I2C2_SCL_PIN * 2));
    GPIOB_PUPDR   |= (1U << (I2C2_SCL_PIN * 2)); // Pull-up
    GPIOB_AFRH    &= ~(0xF << ((I2C2_SCL_PIN - 8) * 4)); 
    GPIOB_AFRH    |= (GPIO_AF4 << ((I2C2_SCL_PIN - 8) * 4)); 

    // 3. Configure PB3 (SDA) -> AF09
    GPIOB_MODER   &= ~(3U << (I2C2_SDA_PIN * 2));
    GPIOB_MODER   |= (2U << (I2C2_SDA_PIN * 2)); // AF Mode
    GPIOB_OTYPER  |= (1U << I2C2_SDA_PIN);       // Open Drain
    GPIOB_OSPEEDR |= (2U << (I2C2_SDA_PIN * 2)); // High Speed
    GPIOB_PUPDR   &= ~(3U << (I2C2_SDA_PIN * 2));
    GPIOB_PUPDR   |= (1U << (I2C2_SDA_PIN * 2)); // Pull-up
    GPIOB_AFRL    &= ~(0xF << (I2C2_SDA_PIN * 4)); 
    GPIOB_AFRL    |= (GPIO_AF9 << (I2C2_SDA_PIN * 4)); 

    // 4. Reset & Config I2C2
    RCC_APB1RSTR |= RCC_APB1RSTR_I2C2RST;
    RCC_APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

    I2C2_CR1 &= ~I2C_CR1_PE; // Disable before config
    
    // Timing calculations for 42MHz APB1 Clock
    // FREQ = 42MHz
    I2C2_CR2 &= ~(0x3F);
    I2C2_CR2 |= 42;          
    
    // CCR for 100kHz: 42MHz / (2 * 100kHz) = 210
    I2C2_CCR = 210;          
    
    // TRISE: (1000ns / 23.8ns) + 1 = 43
    I2C2_TRISE = 43;         
    
    I2C2_CR1 |= I2C_CR1_PE;  // Enable Peripheral
}

// --- HW: Stop Condition ---
static void i2c2_start(void) 
{
    I2C2_CR1 |= I2C_CR1_ACK;
    I2C2_CR1 |= I2C_CR1_START;
}

// --- HW: Stop Condition ---
static void i2c2_stop(void) 
{
    I2C2_CR1 |= I2C_CR1_STOP;
}

// --- HW: Write ---
static void i2c2_write(uint8_t addr, uint8_t *data, uint16_t len) {
    i2c2_start();
    
    while (!(I2C2_SR1 & I2C_SR1_SB)); // Wait for Start Bit

    I2C2_DR = addr; 
    while (!(I2C2_SR1 & I2C_SR1_ADDR)); // Wait for Address matched
    (void)I2C2_SR2; // Clear ADDR flag

    for (uint16_t i = 0; i < len; i++) {
        while (!(I2C2_SR1 & I2C_SR1_TXE)); // Wait for TX Empty
        I2C2_DR = data[i];
    }
    
    while (!(I2C2_SR1 & I2C_SR1_BTF)); // Wait for Byte Transfer Finish

    i2c2_stop();
}

// --- HW: Read ---
static void i2c2_read(uint8_t addr, uint8_t *data, uint16_t len)
{
    i2c2_start();

    while (!(I2C2_SR1 & I2C_SR1_SB)); 

    I2C2_DR = addr | 0x01; // read mode
    while (!(I2C2_SR1 & I2C_SR1_ADDR)); 
    (void)I2C2_SR2; // clear ADDR flag

    for (uint16_t i = 0; i < len; i++) {
        // If last byte, send NACK
        if (i == (len - 1)) {
            I2C2_CR1 &= ~I2C_CR1_ACK;
        } else {
            I2C2_CR1 |= I2C_CR1_ACK;
        }
        
        while (!(I2C2_SR1 & I2C_SR1_RXNE));
        data[i] = (uint8_t)I2C2_DR;
    }
    
    i2c2_stop();
}

/**********************  AHT10 Logic Functions ************************/

// Trigger data measurement
static void logic_trigger(aht10_t *dev)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00}; 
    dev->i2c.write(dev->i2c_addr, cmd, sizeof(cmd));
}

// Soft reset
static void logic_reset(aht10_t *dev) {
    uint8_t cmd = 0xBA;
    dev->i2c.write(dev->i2c_addr, &cmd, 1);
    dev->i2c.delay(20);
}

// Read Raw Data (6 Bytes)
static void logic_read_raw(aht10_t *dev) {
    dev->i2c.read(dev->i2c_addr, dev->raw_data, sizeof(dev->raw_data));
}

// Parse Data (Convert Bytes -> Float)
static void logic_parse(aht10_t *dev) {
    // Humidity: 20-bit (Byte1, Byte2, Top 4 bits of Byte3)
    uint32_t raw_h = ((uint32_t)dev->raw_data[1] << 12) | 
                     ((uint32_t)dev->raw_data[2] << 4) | 
                     ((uint32_t)dev->raw_data[3] >> 4);

    // Temperature: 20-bit (Bottom 4 bits of Byte3, Byte4, Byte5)
    uint32_t raw_t = (((uint32_t)dev->raw_data[3] & 0x0F) << 16) | 
                     ((uint32_t)dev->raw_data[4] << 8) | 
                     (uint32_t)dev->raw_data[5];

    dev->humid_pct = ((float)raw_h / 1048576.0f) * 100.0f;
    dev->temp_c    = (((float)raw_t / 1048576.0f) * 200.0f) - 50.0f;
}


/**********************  PUBLIC API ************************/

void aht10_init(aht10_t *dev) {
    // 1. Bind Hardware Callbacks
    dev->i2c.init  = i2c2_init;
    dev->i2c.stop  = i2c2_stop;
    dev->i2c.write = i2c2_write;
    dev->i2c.read  = i2c2_read;
    dev->i2c.delay = delay_ms;
    
    // 2. Bind Logic Callbacks
    dev->ops.reset    = logic_reset;
    dev->ops.trigger  = logic_trigger;
    dev->ops.read_raw = logic_read_raw;
    dev->ops.parse    = logic_parse;
    
    // 3. Set Defaults
    dev->i2c_addr = AHT10_ADDR_DEFAULT;

    // 4. Run Execution
    dev->i2c.init();
    dev->i2c.delay(100); // Power-on delay
    
    // 5. Send Calibrate Command (0xE1)
    uint8_t cmd[3] = {0xE1, 0x08, 0x00};
    dev->i2c.write(dev->i2c_addr, cmd, sizeof(cmd));
    dev->i2c.delay(10);
}

void aht10_run_cycle(aht10_t *dev) {
    dev->ops.trigger(dev);
    dev->i2c.delay(80); // Wait for measurement (AHT10 needs ~75ms+)
    dev->ops.read_raw(dev);
    dev->ops.parse(dev);
}