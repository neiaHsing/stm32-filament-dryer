#ifndef W25QXX_H
#define W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool W25Q_Init(SPI_HandleTypeDef *spi);
uint32_t W25Q_GetJedecId(void);
uint32_t W25Q_GetCapacityBytes(void);
bool W25Q_Read(uint32_t address, uint8_t *data, uint16_t length);
bool W25Q_PageProgram(uint32_t address, const uint8_t *data, uint16_t length);
bool W25Q_EraseSector(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* W25QXX_H */
