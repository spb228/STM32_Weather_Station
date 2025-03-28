#include "aht10_i2c.h"
#include "common/uart/uart.h"

/*
    * I2C1 SCL -> PB8 -> AF04
    * I2C1 SDA -> PB9 -> AF04
    * 3.3V -> CN7 third one down on first column
    * GND -> CN7 4th one down on second column
*/

void i2c1_gpio_init(void)
{
    /* enable clock for GPIOB */
    RCC_AHB1ENR |= (1 << 1);

    /* GPIO configuration */
    GPIOB_MODER &= ~((3 << (I2C1_SCL_PIN * 2)) | (3 << (I2C1_SDA_PIN * 2)));
    GPIOB_MODER |= (2 << (I2C1_SCL_PIN * 2)); // PB8 Alt func mode
    GPIOB_MODER |= (2 << (I2C1_SDA_PIN * 2)); // PB8 Alt func mode

    GPIOB_OTYPER |= (1 << I2C1_SCL_PIN); // open drain
    GPIOB_OTYPER |= (1 << I2C1_SDA_PIN); // open drain

    GPIOB_OSPEEDR |= (1 << (I2C1_SCL_PIN * 2)); // medium speed
    GPIOB_OSPEEDR |= (1 << (I2C1_SDA_PIN * 2)); // medium speed

    GPIOB_PUPDR &= ~((3 << (I2C1_SCL_PIN * 2)) | (3 << (I2C1_SDA_PIN * 2)));
    GPIOB_PUPDR |= (1 << (I2C1_SCL_PIN * 2)); // pull up
    GPIOB_PUPDR |= (1 << (I2C1_SDA_PIN * 2)); // pull up

    GPIOB_AFRH &= ~((0xF << ((I2C1_SCL_PIN - 8) * 4)) | (0xF << ((I2C1_SDA_PIN - 8) * 4)));
    GPIOB_AFRH |= (4 << ((I2C1_SCL_PIN - 8) * 4)); // PB8 Alt func 4
    GPIOB_AFRH |= (4 << ((I2C1_SDA_PIN - 8) * 4)); // PB8 Alt func 4
}

void i2c1_config(void)
{
    i2c1_gpio_init();

    /* enable clock for I2C 1 */
    RCC_APB1ENR |= (1 << 21); 

    I2C1_CR1 = 0; 
    I2C1_CR2 = 0; 

    I2C1_CR2 |= (42 << 0); // apb1 clock set to 42Mhz 

    /* 
        * Calculation for CRR:
        * CCR = apb1freq / (2 * desired scl clk fq)
        * 42Mhz / (2 * 100Khz) = 210
    */
    I2C1_CCR = 210; 

    /*
        * Maximum rise time for standard mode
        * In standard mode, max rise time is 1000ns
        * Trise = (Freq in MHz + 1)
    */
    I2C1_TRISE = 43;  // 42MHz + 1

    /* enable i2c interrupts */
    // I2C1_CR2 |= (1 << 9); // ITEVTEN 
    // I2C1_CR2 |= (1 << 8); // ITERREN

    /* enable i2c1 */
    I2C1_CR1 |= (1 << 0);

    /* interrupt setup */
    NVIC_ISER0 |= (1 << I2C1_EV_INTERRUPT_NUM);
    NVIC_ISER1 |= (1 << (I2C1_ER_INTERRUPT_NUM - 32));
}

typedef enum
{
    I2C1_STATE_IDLE,
    I2C1_STATE_START, 
    I2C1_STATE_SEND_ADDR,
    I2C1_STATE_SEND_DATA,
    I2C1_STATE_RESTART,
    I2C1_STATE_RECEIVE_DATA,
    I2C1_STATE_STOP,
    I2C1_STATE_ERROR
} I2C1_State; 

typedef struct 
{
    volatile uint8_t *tx_buffer;
    volatile uint8_t *rx_buffer; 
    volatile uint16_t tx_length; 
    volatile uint16_t rx_length; 
    volatile uint16_t tx_index;
    volatile uint16_t rx_index;
    volatile I2C1_State state; 
    volatile uint8_t device_addr; 
    volatile uint8_t error; 
} I2C1_Transaction_Context; 

volatile I2C1_Transaction_Context i2c1_transaction; 

void I2C1_StartTransaction(
    uint8_t dev_addr, 
    uint8_t *tx_data,
    uint16_t tx_len, 
    uint8_t *rx_data,
    uint16_t rx_len)
{
    print("I2C1 starting transaction");

    //__asm volatile ("cpsid i"); // Disable all interrupts
    
    /* reset all transaction parameters */
    i2c1_transaction.tx_buffer = tx_data; 
    i2c1_transaction.rx_buffer = rx_data; 
    i2c1_transaction.tx_length = tx_len; 
    i2c1_transaction.rx_length = rx_len; 
    i2c1_transaction.tx_index = 0; 
    i2c1_transaction.rx_index = 0; 
    i2c1_transaction.state = I2C1_STATE_START; 
    i2c1_transaction.device_addr = dev_addr;
    i2c1_transaction.error = 0; 

    /* enable i2c interrupts */
    I2C1_CR2 |= (1 << 9); // ITEVTEN 
    I2C1_CR2 |= (1 << 8); // ITERREN

    //__asm volatile ("cpsid i"); // enable all interrupts
}

void I2C1_EV_IRQHandler(void)
{
    print("I2C1 event interrupt triggered..."); 

    /* read SR1 to clear flags */
    volatile uint32_t sr1 = I2C1_SR1;
    volatile uint32_t sr2 = I2C1_SR2; 

    switch(i2c1_transaction.state)
    {
        case I2C1_STATE_START:
            /* start condition generated, send address */
            I2C1_DR = i2c1_transaction.device_addr;
            i2c1_transaction.state = I2C1_STATE_SEND_ADDR; 
            break; 

        case I2C1_STATE_SEND_ADDR:
            /* address sent, prep for data transmission */
            if (i2c1_transaction.tx_length > 0)
            {
                i2c1_transaction.state = I2C1_STATE_SEND_DATA; 
            }
            else if (i2c1_transaction.rx_length > 0)
            {
                /* prepare for reading */
                I2C1_CR1 |= I2C1_CR1_START; /* TODO */
                i2c1_transaction.state = I2C1_STATE_RESTART; 
            }
            else 
            {
                /* no data to send or receive */
                I2C1_CR1 |= I2C1_CR1_STOP; /* TODO */
                i2c1_transaction.state = I2C1_STATE_IDLE;  
            }
            break; 
        
        case I2C1_STATE_SEND_DATA:
            if (i2c1_transaction.tx_index < i2c1_transaction.tx_length)
            {
                I2C1_DR = i2c1_transaction.tx_buffer[i2c1_transaction.tx_index++]; 
            }
            else if (i2c1_transaction.rx_length > 0)
            {
                /* restart for read after write */
                I2C1_CR1 |= I2C1_CR1_START; 
                i2c1_transaction.state = I2C1_STATE_RESTART; 
            }
            else 
            {
                /* stop transmission */
                I2C1_CR1 |= I2C1_CR1_STOP; /* TODO */
                i2c1_transaction.state = I2C1_STATE_IDLE;  
            }
            break; 
        
        case I2C1_STATE_RESTART:
            /* send address for read */
            I2C1_DR = i2c1_transaction.device_addr | 0x01; // read bit // TODO: Why do this? 
            i2c1_transaction.state = I2C1_STATE_RECEIVE_DATA; 
            break; 
        
        case I2C1_STATE_RECEIVE_DATA:
            if (i2c1_transaction.rx_index < i2c1_transaction.rx_length)
            {
                /* configure ACK */
                if (i2c1_transaction.rx_index == i2c1_transaction.rx_length - 1)
                {
                    /* last byte disable ACK */
                    I2C1_CR1 &= ~I2C1_CR1_ACK; // TODO
                }

                /* read received byte */
                i2c1_transaction.rx_buffer[i2c1_transaction.rx_index] = I2C1_DR;

                print((char *)i2c1_transaction.rx_buffer[i2c1_transaction.rx_index]);

                i2c1_transaction.rx_index++;

                /* check if reception complete */
                if (i2c1_transaction.rx_index >= i2c1_transaction.rx_length)
                {
                    I2C1_CR1 |= I2C1_CR1_STOP; /* TODO */
                    i2c1_transaction.state = I2C1_STATE_IDLE;  
                }
            }
            break; 
        
        default:
            break; 
    }
}

void I2C1_ER_IRQHandler(void)
{
    print("I2C1 error interrupt triggered..."); 
}

