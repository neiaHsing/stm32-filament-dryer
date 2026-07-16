#include "aht20.h"

#define AHT20_ADDRESS          (0x38U << 1U)
#define AHT20_TIMEOUT_MS       100U
#define AHT20_STATUS_BUSY      0x80U
#define AHT20_STATUS_CALIBRATED 0x08U

static I2C_HandleTypeDef *aht20_i2c;

static bool AHT20_ReadStatus(uint8_t *status)
{
  const uint8_t command = 0x71U;
  if (HAL_I2C_Master_Transmit(aht20_i2c, AHT20_ADDRESS, (uint8_t *)&command,
                              1U, AHT20_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  return HAL_I2C_Master_Receive(aht20_i2c, AHT20_ADDRESS, status, 1U,
                                AHT20_TIMEOUT_MS) == HAL_OK;
}

static uint8_t AHT20_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFFU;
  for (uint8_t i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x31U)
                                : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

bool AHT20_Init(I2C_HandleTypeDef *i2c)
{
  if (i2c == NULL)
  {
    return false;
  }

  aht20_i2c = i2c;
  HAL_Delay(40U);

  if (HAL_I2C_IsDeviceReady(aht20_i2c, AHT20_ADDRESS, 3U,
                            AHT20_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  uint8_t status = 0U;
  if (!AHT20_ReadStatus(&status))
  {
    return false;
  }

  if ((status & AHT20_STATUS_CALIBRATED) == 0U)
  {
    uint8_t initialize[] = {0xBEU, 0x08U, 0x00U};
    if (HAL_I2C_Master_Transmit(aht20_i2c, AHT20_ADDRESS, initialize,
                                sizeof(initialize), AHT20_TIMEOUT_MS) != HAL_OK)
    {
      return false;
    }
    HAL_Delay(10U);

    if (!AHT20_ReadStatus(&status) ||
        (status & AHT20_STATUS_CALIBRATED) == 0U)
    {
      return false;
    }
  }

  return true;
}

bool AHT20_StartMeasurement(void)
{
  if (aht20_i2c == NULL)
  {
    return false;
  }

  uint8_t trigger[] = {0xACU, 0x33U, 0x00U};
  return HAL_I2C_Master_Transmit(aht20_i2c, AHT20_ADDRESS, trigger,
                                 sizeof(trigger), AHT20_TIMEOUT_MS) == HAL_OK;
}

bool AHT20_ReadMeasurement(AHT20_Measurement *measurement)
{
  if (aht20_i2c == NULL || measurement == NULL)
  {
    return false;
  }

  uint8_t data[7] = {0U};
  if (HAL_I2C_Master_Receive(aht20_i2c, AHT20_ADDRESS, data, sizeof(data),
                             AHT20_TIMEOUT_MS) != HAL_OK ||
      (data[0] & AHT20_STATUS_BUSY) != 0U ||
      AHT20_Crc8(data, 6U) != data[6])
  {
    return false;
  }

  const uint32_t raw_humidity = ((uint32_t)data[1] << 12U) |
                                ((uint32_t)data[2] << 4U) |
                                ((uint32_t)data[3] >> 4U);
  const uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0FU) << 16U) |
                                   ((uint32_t)data[4] << 8U) |
                                   data[5];

  measurement->humidity_milli_percent =
      (uint32_t)(((uint64_t)raw_humidity * 100000ULL) >> 20U);
  measurement->temperature_centi_c =
      (int32_t)(((uint64_t)raw_temperature * 20000ULL) >> 20U) - 5000;

  if (measurement->humidity_milli_percent > 100000U)
  {
    measurement->humidity_milli_percent = 100000U;
  }
  return true;
}
