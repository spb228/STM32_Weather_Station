# STM32_Weather_Station
### Project Description
This weather station can monitor the temperature and humidity of a space while displaying the data on a small LCD screen. The data and logs from the sensors are stored in a microSD card for post-analysis. 

### Hardware
* STM32f401re Nucleo Development Board
* LCD Display (I2C) : SSD1306
* Temp/Humidity Sensor (I2C) : AHT10
* MicroSD Adapter/Module (SPI) : Generic model

### Building and Running the Project
* Clone the repo
* Create a "build" directory inside the repo
* Run make from inside the repo's root level using "mingw32-make clean all"
* Upload the binary file from the build folder to the MC (VSCode)
