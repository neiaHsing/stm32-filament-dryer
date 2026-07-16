#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool BMP280_Init(I2C_HandleTypeDef *i2c);
bool BMP280_ReadTemperature(int32_t *temperature_centi_c);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
