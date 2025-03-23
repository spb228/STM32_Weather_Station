# STM32F401RE Weather Station Data Logger Blueprint

## Hardware Components
- **Microcontroller**: STM32F401RE Nucleo Dev Board
- **Display**: SSD1306 (I2C)
- **Temp/Humidity Sensor**: AHT10 (I2C)
- **MicroSD adapter/module** (SPI) for data storage

## System Architecture

### Hardware Interfaces
- STM32F401RE ↔ SSD1306 Display (I2C1)
- STM32F401RE ↔ AHT10 Sensor (I2C1, shared bus)
- STM32F401RE ↔ MicroSD Card (SPI1)

### Software Layers
1. **Hardware Abstraction Layer (HAL)**
   - Direct register manipulation for each peripheral
   - No standard libraries/HALs to maintain bare metal approach

2. **Driver Layer**
   - I2C driver for display and sensor
   - SPI driver for SD card
   - FatFS implementation for file system

3. **Application Layer**
   - Sensor data acquisition
   - Display controller
   - Data logging subsystem
   - Real-time clock management

## Implementation Roadmap

### 1. System Initialization
- Configure system clock (84MHz from 8MHz HSE)
- Initialize GPIO for status LEDs
- Configure I2C1 for sensor and display
- Configure SPI1 for SD card