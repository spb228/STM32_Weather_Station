#ifndef USD_CARD_H
#define USD_CARD_H

#include <stdint.h>
#include <stdbool.h>
#include "common_def.h"
#include "uart.h"

/*

GPIO Assignments:

PA4 -> CN7-32 -> CS (manual config to CS)
PA5 -> CN10-11 -> SCK (AF5)
PA6 -> CN10-13 -> MISO (AF5)
PA7 -> CN10-15 -> MOSI (AF5)

*/

void SPI1_GPIO_Init(void);
void SPI1_Init(void);
uint8_t SPI1_Receive(void);

void SD_Init(void);
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
void SD_SendDummyClocks(void);
uint8_t SD_Send_CMD8(void);

#endif /* USD_CARD_H*/