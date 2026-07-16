#include "w25qxx.h"

#include "main.h"

#define W25Q_CMD_WRITE_ENABLE       0x06U
#define W25Q_CMD_READ_STATUS_1      0x05U
#define W25Q_CMD_READ_DATA          0x03U
#define W25Q_CMD_PAGE_PROGRAM       0x02U
#define W25Q_CMD_SECTOR_ERASE       0x20U
#define W25Q_CMD_JEDEC_ID           0x9FU
#define W25Q_CMD_RELEASE_POWER_DOWN 0xABU

#define W25Q_STATUS_BUSY            0x01U
#define W25Q_STATUS_WRITE_ENABLE    0x02U
#define W25Q_PAGE_SIZE              256U
#define W25Q_SECTOR_SIZE            4096U
#define W25Q_SPI_TIMEOUT_MS         100U
#define W25Q_PROGRAM_TIMEOUT_MS     1000U
#define W25Q_ERASE_TIMEOUT_MS       5000U
#define W25Q_MANUFACTURER_WINBOND   0xEFU

static SPI_HandleTypeDef *w25q_spi;
static uint32_t w25q_jedec_id;
static uint32_t w25q_capacity_bytes;

static void W25Q_Select(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void W25Q_Deselect(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static bool W25Q_Transmit(const uint8_t *data, uint16_t length)
{
  return HAL_SPI_Transmit(w25q_spi, (uint8_t *)data, length,
                          W25Q_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool W25Q_Receive(uint8_t *data, uint16_t length)
{
  return HAL_SPI_Receive(w25q_spi, data, length,
                         W25Q_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool W25Q_SendCommand(uint8_t command)
{
  W25Q_Select();
  const bool success = W25Q_Transmit(&command, 1U);
  W25Q_Deselect();
  return success;
}

static bool W25Q_ReadStatus(uint8_t *status)
{
  const uint8_t command = W25Q_CMD_READ_STATUS_1;
  W25Q_Select();
  const bool success = W25Q_Transmit(&command, 1U) &&
                       W25Q_Receive(status, 1U);
  W25Q_Deselect();
  return success;
}

static bool W25Q_WaitReady(uint32_t timeout_ms)
{
  const uint32_t started = HAL_GetTick();
  uint8_t status = 0U;
  do
  {
    if (!W25Q_ReadStatus(&status))
    {
      return false;
    }
    if ((status & W25Q_STATUS_BUSY) == 0U)
    {
      return true;
    }
  } while (HAL_GetTick() - started < timeout_ms);
  return false;
}

static bool W25Q_WriteEnable(void)
{
  uint8_t status = 0U;
  return W25Q_SendCommand(W25Q_CMD_WRITE_ENABLE) &&
         W25Q_ReadStatus(&status) &&
         (status & W25Q_STATUS_WRITE_ENABLE) != 0U;
}

bool W25Q_Init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL)
  {
    return false;
  }

  w25q_spi = spi;
  W25Q_Deselect();
  W25Q_SendCommand(W25Q_CMD_RELEASE_POWER_DOWN);
  HAL_Delay(1U);

  const uint8_t command = W25Q_CMD_JEDEC_ID;
  uint8_t id[3] = {0U};
  W25Q_Select();
  const bool success = W25Q_Transmit(&command, 1U) && W25Q_Receive(id, 3U);
  W25Q_Deselect();
  if (!success)
  {
    return false;
  }

  w25q_jedec_id = ((uint32_t)id[0] << 16U) |
                  ((uint32_t)id[1] << 8U) | id[2];
  const uint8_t capacity_code = id[2];
  if (id[0] != W25Q_MANUFACTURER_WINBOND ||
      capacity_code < 0x11U || capacity_code > 0x18U)
  {
    w25q_jedec_id = 0U;
    return false;
  }

  w25q_capacity_bytes = 1UL << capacity_code;
  return W25Q_WaitReady(W25Q_SPI_TIMEOUT_MS);
}

uint32_t W25Q_GetJedecId(void)
{
  return w25q_jedec_id;
}

uint32_t W25Q_GetCapacityBytes(void)
{
  return w25q_capacity_bytes;
}

bool W25Q_Read(uint32_t address, uint8_t *data, uint16_t length)
{
  if (w25q_spi == NULL || data == NULL || length == 0U ||
      address >= w25q_capacity_bytes ||
      length > w25q_capacity_bytes - address ||
      !W25Q_WaitReady(W25Q_SPI_TIMEOUT_MS))
  {
    return false;
  }

  const uint8_t header[4] = {
      W25Q_CMD_READ_DATA,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)address,
  };
  W25Q_Select();
  const bool success = W25Q_Transmit(header, sizeof(header)) &&
                       W25Q_Receive(data, length);
  W25Q_Deselect();
  return success;
}

bool W25Q_PageProgram(uint32_t address, const uint8_t *data, uint16_t length)
{
  if (w25q_spi == NULL || data == NULL || length == 0U ||
      address >= w25q_capacity_bytes ||
      length > w25q_capacity_bytes - address)
  {
    return false;
  }

  while (length > 0U)
  {
    uint16_t chunk = (uint16_t)(W25Q_PAGE_SIZE -
                                (address & (W25Q_PAGE_SIZE - 1U)));
    if (chunk > length)
    {
      chunk = length;
    }

    if (!W25Q_WaitReady(W25Q_PROGRAM_TIMEOUT_MS) || !W25Q_WriteEnable())
    {
      return false;
    }

    const uint8_t header[4] = {
        W25Q_CMD_PAGE_PROGRAM,
        (uint8_t)(address >> 16U),
        (uint8_t)(address >> 8U),
        (uint8_t)address,
    };
    W25Q_Select();
    const bool success = W25Q_Transmit(header, sizeof(header)) &&
                         W25Q_Transmit(data, chunk);
    W25Q_Deselect();
    if (!success || !W25Q_WaitReady(W25Q_PROGRAM_TIMEOUT_MS))
    {
      return false;
    }

    address += chunk;
    data += chunk;
    length -= chunk;
  }
  return true;
}

bool W25Q_EraseSector(uint32_t address)
{
  if (w25q_spi == NULL || address >= w25q_capacity_bytes ||
      !W25Q_WaitReady(W25Q_ERASE_TIMEOUT_MS) || !W25Q_WriteEnable())
  {
    return false;
  }

  address &= ~(W25Q_SECTOR_SIZE - 1U);
  const uint8_t command[4] = {
      W25Q_CMD_SECTOR_ERASE,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)address,
  };
  W25Q_Select();
  const bool success = W25Q_Transmit(command, sizeof(command));
  W25Q_Deselect();
  return success && W25Q_WaitReady(W25Q_ERASE_TIMEOUT_MS);
}
