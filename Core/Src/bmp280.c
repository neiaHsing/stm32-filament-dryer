#include "bmp280.h"

#define BMP280_ADDRESS_LOW       (0x76U << 1U)
#define BMP280_ADDRESS_HIGH      (0x77U << 1U)
#define BMP280_CHIP_ID_REGISTER  0xD0U
#define BMP280_CHIP_ID           0x58U
#define BMP280_CALIB_REGISTER    0x88U
#define BMP280_CONFIG_REGISTER   0xF5U
#define BMP280_CTRL_REGISTER     0xF4U
#define BMP280_TEMP_REGISTER     0xFAU
#define BMP280_TIMEOUT_MS        100U

typedef struct
{
  uint16_t t1;
  int16_t t2;
  int16_t t3;
} BMP280_Calibration;

static I2C_HandleTypeDef *bmp280_i2c;
static uint16_t bmp280_address;
static BMP280_Calibration bmp280_calibration;

static bool BMP280_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  return HAL_I2C_Mem_Read(bmp280_i2c, bmp280_address, reg,
                          I2C_MEMADD_SIZE_8BIT, data, length,
                          BMP280_TIMEOUT_MS) == HAL_OK;
}

static bool BMP280_Write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(bmp280_i2c, bmp280_address, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           BMP280_TIMEOUT_MS) == HAL_OK;
}

bool BMP280_Init(I2C_HandleTypeDef *i2c)
{
  if (i2c == NULL)
  {
    return false;
  }

  bmp280_i2c = i2c;
  const uint16_t addresses[] = {BMP280_ADDRESS_LOW, BMP280_ADDRESS_HIGH};
  uint8_t chip_id = 0U;

  for (uint8_t i = 0U; i < 2U; ++i)
  {
    bmp280_address = addresses[i];
    if (HAL_I2C_IsDeviceReady(bmp280_i2c, bmp280_address, 2U,
                              BMP280_TIMEOUT_MS) == HAL_OK &&
        BMP280_Read(BMP280_CHIP_ID_REGISTER, &chip_id, 1U) &&
        chip_id == BMP280_CHIP_ID)
    {
      break;
    }
    chip_id = 0U;
  }

  if (chip_id != BMP280_CHIP_ID)
  {
    return false;
  }

  uint8_t calibration[6] = {0U};
  if (!BMP280_Read(BMP280_CALIB_REGISTER, calibration, sizeof(calibration)))
  {
    return false;
  }

  bmp280_calibration.t1 = (uint16_t)((uint16_t)calibration[1] << 8U) |
                          calibration[0];
  bmp280_calibration.t2 = (int16_t)((uint16_t)calibration[3] << 8U) |
                          calibration[2];
  bmp280_calibration.t3 = (int16_t)((uint16_t)calibration[5] << 8U) |
                          calibration[4];

  /* 1000 ms standby, temperature x2, pressure skipped, normal mode. */
  if (!BMP280_Write(BMP280_CONFIG_REGISTER, 0xA0U) ||
      !BMP280_Write(BMP280_CTRL_REGISTER, 0x43U))
  {
    return false;
  }
  HAL_Delay(10U);
  return true;
}

bool BMP280_ReadTemperature(int32_t *temperature_centi_c)
{
  if (bmp280_i2c == NULL || temperature_centi_c == NULL)
  {
    return false;
  }

  uint8_t data[3] = {0U};
  if (!BMP280_Read(BMP280_TEMP_REGISTER, data, sizeof(data)))
  {
    return false;
  }

  const int32_t adc_temperature = ((int32_t)data[0] << 12U) |
                                  ((int32_t)data[1] << 4U) |
                                  ((int32_t)data[2] >> 4U);
  if (adc_temperature == 0x80000L)
  {
    return false;
  }

  const int32_t var1 = (((adc_temperature >> 3) -
                        ((int32_t)bmp280_calibration.t1 << 1)) *
                        (int32_t)bmp280_calibration.t2) >> 11;
  const int32_t delta = (adc_temperature >> 4) -
                        (int32_t)bmp280_calibration.t1;
  const int32_t var2 = (((delta * delta) >> 12) *
                        (int32_t)bmp280_calibration.t3) >> 14;
  const int32_t fine_temperature = var1 + var2;
  *temperature_centi_c = (fine_temperature * 5 + 128) >> 8;
  return true;
}
