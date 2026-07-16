#include "settings_storage.h"

#include "w25qxx.h"

#include <string.h>

#define STORAGE_SECTOR_SIZE       4096U
#define STORAGE_SECTOR_COUNT      2U
#define STORAGE_RECORD_SIZE       16U
#define STORAGE_RECORDS_PER_SECTOR (STORAGE_SECTOR_SIZE / STORAGE_RECORD_SIZE)
#define STORAGE_MAGIC_0           'D'
#define STORAGE_MAGIC_1           'R'
#define STORAGE_MAGIC_2           'Y'
#define STORAGE_MAGIC_3           '1'
#define STORAGE_VERSION_LEGACY    1U
#define STORAGE_VERSION           2U
#define STORAGE_FLAG_TEMPERATURE  0x01U
#define STORAGE_FLAG_HUMIDITY     0x02U

static bool storage_initialized;
static bool storage_has_settings;
static uint32_t storage_sector_base[STORAGE_SECTOR_COUNT];
static uint8_t storage_active_sector;
static uint16_t storage_next_slot;
static uint32_t storage_sequence;
static StoredSettings storage_current;

static uint32_t Storage_ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static void Storage_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static uint32_t Storage_Crc32(const uint8_t *data, uint8_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint8_t i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320UL
                              : crc >> 1U;
    }
  }
  return ~crc;
}

static bool Storage_RecordErased(const uint8_t record[STORAGE_RECORD_SIZE])
{
  for (uint8_t i = 0U; i < STORAGE_RECORD_SIZE; ++i)
  {
    if (record[i] != 0xFFU)
    {
      return false;
    }
  }
  return true;
}

static bool Storage_RecordValid(const uint8_t record[STORAGE_RECORD_SIZE])
{
  const bool legacy_metadata =
      record[10] == STORAGE_VERSION_LEGACY &&
      record[11] == (uint8_t)(record[8] ^ record[9] ^ 0xA5U);
  const bool current_metadata =
      record[10] == STORAGE_VERSION &&
      (record[11] &
       (uint8_t)~(STORAGE_FLAG_TEMPERATURE | STORAGE_FLAG_HUMIDITY)) == 0U;

  return record[0] == STORAGE_MAGIC_0 &&
         record[1] == STORAGE_MAGIC_1 &&
         record[2] == STORAGE_MAGIC_2 &&
         record[3] == STORAGE_MAGIC_3 &&
         (legacy_metadata || current_metadata) &&
         Storage_ReadU32(&record[12]) == Storage_Crc32(record, 12U);
}

static bool Storage_SequenceNewer(uint32_t candidate, uint32_t current)
{
  return (int32_t)(candidate - current) > 0;
}

static bool Storage_FindErasedSlot(uint8_t sector, uint16_t first_slot,
                                   uint16_t *slot)
{
  uint8_t record[STORAGE_RECORD_SIZE];
  for (uint16_t index = first_slot; index < STORAGE_RECORDS_PER_SECTOR;
       ++index)
  {
    const uint32_t address = storage_sector_base[sector] +
                             (uint32_t)index * STORAGE_RECORD_SIZE;
    if (!W25Q_Read(address, record, sizeof(record)))
    {
      return false;
    }
    if (Storage_RecordErased(record))
    {
      *slot = index;
      return true;
    }
  }
  *slot = STORAGE_RECORDS_PER_SECTOR;
  return true;
}

bool SettingsStorage_Init(SPI_HandleTypeDef *spi)
{
  storage_initialized = false;
  storage_has_settings = false;
  if (!W25Q_Init(spi))
  {
    return false;
  }

  const uint32_t capacity = W25Q_GetCapacityBytes();
  if (capacity < STORAGE_SECTOR_SIZE * STORAGE_SECTOR_COUNT)
  {
    return false;
  }
  storage_sector_base[0] = capacity -
                           STORAGE_SECTOR_SIZE * STORAGE_SECTOR_COUNT;
  storage_sector_base[1] = capacity - STORAGE_SECTOR_SIZE;

  uint8_t latest_sector = 0U;
  uint16_t latest_slot = 0U;
  uint8_t record[STORAGE_RECORD_SIZE];
  for (uint8_t sector = 0U; sector < STORAGE_SECTOR_COUNT; ++sector)
  {
    for (uint16_t slot = 0U; slot < STORAGE_RECORDS_PER_SECTOR; ++slot)
    {
      const uint32_t address = storage_sector_base[sector] +
                               (uint32_t)slot * STORAGE_RECORD_SIZE;
      if (!W25Q_Read(address, record, sizeof(record)))
      {
        return false;
      }
      if (!Storage_RecordValid(record))
      {
        continue;
      }

      const uint32_t sequence = Storage_ReadU32(&record[4]);
      if (!storage_has_settings ||
          Storage_SequenceNewer(sequence, storage_sequence))
      {
        storage_has_settings = true;
        storage_sequence = sequence;
        storage_current.target_temperature_c = record[8];
        storage_current.target_humidity_percent = record[9];
        if (record[10] == STORAGE_VERSION_LEGACY)
        {
          storage_current.temperature_enabled = true;
          storage_current.humidity_enabled = true;
        }
        else
        {
          storage_current.temperature_enabled =
              (record[11] & STORAGE_FLAG_TEMPERATURE) != 0U;
          storage_current.humidity_enabled =
              (record[11] & STORAGE_FLAG_HUMIDITY) != 0U;
        }
        latest_sector = sector;
        latest_slot = slot;
      }
    }
  }

  storage_active_sector = storage_has_settings ? latest_sector : 0U;
  const uint16_t first_slot = storage_has_settings
                                  ? (uint16_t)(latest_slot + 1U)
                                  : 0U;
  if (!Storage_FindErasedSlot(storage_active_sector, first_slot,
                              &storage_next_slot))
  {
    return false;
  }

  storage_initialized = true;
  return true;
}

bool SettingsStorage_Load(StoredSettings *settings)
{
  if (!storage_initialized || !storage_has_settings || settings == NULL)
  {
    return false;
  }
  *settings = storage_current;
  return true;
}

bool SettingsStorage_Save(const StoredSettings *settings)
{
  if (!storage_initialized || settings == NULL)
  {
    return false;
  }
  if (storage_has_settings &&
      settings->target_temperature_c == storage_current.target_temperature_c &&
      settings->target_humidity_percent ==
          storage_current.target_humidity_percent &&
      settings->temperature_enabled == storage_current.temperature_enabled &&
      settings->humidity_enabled == storage_current.humidity_enabled)
  {
    return true;
  }

  uint8_t target_sector = storage_active_sector;
  uint16_t target_slot = storage_next_slot;
  if (target_slot >= STORAGE_RECORDS_PER_SECTOR)
  {
    target_sector = (uint8_t)(1U - storage_active_sector);
    if (!W25Q_EraseSector(storage_sector_base[target_sector]))
    {
      return false;
    }
    target_slot = 0U;
  }

  uint8_t record[STORAGE_RECORD_SIZE];
  memset(record, 0xFF, sizeof(record));
  record[0] = STORAGE_MAGIC_0;
  record[1] = STORAGE_MAGIC_1;
  record[2] = STORAGE_MAGIC_2;
  record[3] = STORAGE_MAGIC_3;
  const uint32_t next_sequence = storage_has_settings
                                     ? storage_sequence + 1U
                                     : 1U;
  Storage_WriteU32(&record[4], next_sequence);
  record[8] = settings->target_temperature_c;
  record[9] = settings->target_humidity_percent;
  record[10] = STORAGE_VERSION;
  record[11] = (settings->temperature_enabled
                    ? STORAGE_FLAG_TEMPERATURE
                    : 0U) |
               (settings->humidity_enabled ? STORAGE_FLAG_HUMIDITY : 0U);
  Storage_WriteU32(&record[12], Storage_Crc32(record, 12U));

  const uint32_t address = storage_sector_base[target_sector] +
                           (uint32_t)target_slot * STORAGE_RECORD_SIZE;
  uint8_t verify[STORAGE_RECORD_SIZE];
  if (!W25Q_PageProgram(address, record, sizeof(record)) ||
      !W25Q_Read(address, verify, sizeof(verify)) ||
      memcmp(record, verify, sizeof(record)) != 0 ||
      !Storage_RecordValid(verify))
  {
    return false;
  }

  storage_active_sector = target_sector;
  storage_sequence = next_sequence;
  storage_current = *settings;
  storage_has_settings = true;
  if (!Storage_FindErasedSlot(storage_active_sector,
                              (uint16_t)(target_slot + 1U),
                              &storage_next_slot))
  {
    storage_next_slot = STORAGE_RECORDS_PER_SECTOR;
  }
  return true;
}
