#ifndef SETTINGS_STORAGE_H
#define SETTINGS_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "ambient_temperature.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t target_temperature_c;
  uint8_t target_humidity_percent;
  bool temperature_enabled;
  bool humidity_enabled;
  int8_t ambient_temperature_c;
} StoredSettings;

bool SettingsStorage_Init(SPI_HandleTypeDef *spi);
bool SettingsStorage_Load(StoredSettings *settings);
bool SettingsStorage_Save(const StoredSettings *settings);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_STORAGE_H */
