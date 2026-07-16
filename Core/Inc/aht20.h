#ifndef AHT20_H
#define AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t temperature_centi_c;
  uint32_t humidity_milli_percent;
} AHT20_Measurement;

bool AHT20_Init(I2C_HandleTypeDef *i2c);
bool AHT20_StartMeasurement(void);
bool AHT20_ReadMeasurement(AHT20_Measurement *measurement);

#ifdef __cplusplus
}
#endif

#endif /* AHT20_H */
