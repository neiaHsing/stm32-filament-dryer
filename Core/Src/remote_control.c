#include "remote_control.h"

#include <stddef.h>

static uint32_t parse_error_count;
static RemoteControlCommand pending_command;
static bool command_pending;

static void RecordError(void)
{
  if (parse_error_count != UINT32_MAX)
  {
    ++parse_error_count;
  }
}

static uint16_t Crc16CcittFalse(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static bool ParseU32(const uint8_t *begin, const uint8_t *end,
                     uint32_t *value)
{
  if (begin == end)
  {
    return false;
  }

  uint32_t parsed = 0U;
  for (const uint8_t *cursor = begin; cursor != end; ++cursor)
  {
    if (*cursor < (uint8_t)'0' || *cursor > (uint8_t)'9')
    {
      return false;
    }
    const uint32_t digit = (uint32_t)(*cursor - (uint8_t)'0');
    if (parsed > (UINT32_MAX - digit) / 10U)
    {
      return false;
    }
    parsed = parsed * 10U + digit;
  }
  *value = parsed;
  return true;
}

static bool ParseI32(const uint8_t *begin, const uint8_t *end,
                     int32_t *value)
{
  if (begin == end)
  {
    return false;
  }
  bool negative = false;
  if (*begin == (uint8_t)'-')
  {
    negative = true;
    ++begin;
  }
  uint32_t magnitude = 0U;
  if (begin == end || !ParseU32(begin, end, &magnitude) || magnitude > 2147483648U)
  {
    return false;
  }
  if (negative)
  {
    *value = magnitude == 2147483648U ? INT32_MIN : -(int32_t)magnitude;
  }
  else
  {
    *value = (int32_t)magnitude;
  }
  return true;
}

static bool ParseFlag(const uint8_t *begin, const uint8_t *end, bool *value)
{
  if (end - begin != 1 || (*begin != (uint8_t)'0' &&
                            *begin != (uint8_t)'1'))
  {
    return false;
  }
  *value = *begin == (uint8_t)'1';
  return true;
}

static int8_t HexDigit(uint8_t value)
{
  if (value >= (uint8_t)'0' && value <= (uint8_t)'9')
  {
    return (int8_t)(value - (uint8_t)'0');
  }
  if (value >= (uint8_t)'A' && value <= (uint8_t)'F')
  {
    return (int8_t)(value - (uint8_t)'A' + 10U);
  }
  if (value >= (uint8_t)'a' && value <= (uint8_t)'f')
  {
    return (int8_t)(value - (uint8_t)'a' + 10U);
  }
  return -1;
}

static bool ParseFrame(const uint8_t *frame, size_t length,
                       RemoteControlCommand *command)
{
  if (length < 2U || frame[length - 2U] != (uint8_t)'\r' ||
      frame[length - 1U] != (uint8_t)'\n')
  {
    return false;
  }

  const size_t content_length = length - 2U;
  size_t star_index = content_length;
  for (size_t index = 0U; index < content_length; ++index)
  {
    if (frame[index] == (uint8_t)'*')
    {
      if (star_index != content_length)
      {
        return false;
      }
      star_index = index;
    }
  }
  if (star_index == content_length ||
      content_length - star_index - 1U != 4U || star_index == 0U)
  {
    return false;
  }

  uint16_t received_crc = 0U;
  for (size_t index = star_index + 1U; index < content_length; ++index)
  {
    const int8_t digit = HexDigit(frame[index]);
    if (digit < 0)
    {
      return false;
    }
    received_crc = (uint16_t)((received_crc << 4U) | (uint8_t)digit);
  }
  if (received_crc != Crc16CcittFalse(frame, star_index))
  {
    return false;
  }

  if (frame[0] == (uint8_t)'P')
  {
    const uint8_t *fields_begin[6] = {0};
    const uint8_t *fields_end[6] = {0};
    size_t field_count = 0U;
    const uint8_t *field_begin = frame;
    for (size_t index = 0U; index <= star_index; ++index)
    {
      if (index == star_index || frame[index] == (uint8_t)',')
      {
        if (field_count >= 6U)
        {
          return false;
        }
        fields_begin[field_count] = field_begin;
        fields_end[field_count] = &frame[index];
        ++field_count;
        field_begin = &frame[index + 1U];
      }
    }
    RemoteControlCommand parsed = {0};
    uint32_t kp = 0U;
    uint32_t ki = 0U;
    uint32_t kd = 0U;
    if (field_count != 6U || fields_end[0] - fields_begin[0] != 1 ||
        fields_begin[0][0] != (uint8_t)'P' ||
        !ParseU32(fields_begin[1], fields_end[1], &parsed.session) ||
        !ParseU32(fields_begin[2], fields_end[2], &parsed.sequence) ||
        !ParseU32(fields_begin[3], fields_end[3], &kp) ||
        !ParseU32(fields_begin[4], fields_end[4], &ki) ||
        !ParseU32(fields_begin[5], fields_end[5], &kd))
    {
      return false;
    }
    parsed.pid_update = true;
    parsed.pid_kp_tenths = (uint16_t)kp;
    parsed.pid_ki_hundredths = (uint16_t)ki;
    parsed.pid_kd_tenths = (uint16_t)kd;
    parsed.values_in_range =
        parsed.session != 0U && parsed.sequence != 0U &&
        kp <= REMOTE_PID_KP_MAX_TENTHS &&
        ki <= REMOTE_PID_KI_MAX_HUNDREDTHS &&
        kd <= REMOTE_PID_KD_MAX_TENTHS;
    *command = parsed;
    return true;
  }

  const uint8_t *fields_begin[12] = {0};
  const uint8_t *fields_end[12] = {0};
  size_t field_count = 0U;
  const uint8_t *field_begin = frame;
  for (size_t index = 0U; index <= star_index; ++index)
  {
    if (index == star_index || frame[index] == (uint8_t)',')
    {
      if (field_count >= 12U)
      {
        return false;
      }
      fields_begin[field_count] = field_begin;
      fields_end[field_count] = &frame[index];
      ++field_count;
      field_begin = &frame[index + 1U];
    }
  }
  if ((field_count != 11U && field_count != 12U) ||
      fields_end[0] - fields_begin[0] != 1 ||
      fields_begin[0][0] != (uint8_t)'C')
  {
    return false;
  }

  RemoteControlCommand parsed = {0};
  uint32_t temperature = 0U;
  uint32_t humidity = 0U;
  int32_t ambient = REMOTE_DEFAULT_AMBIENT_TEMPERATURE_C;
  const size_t clear_fault_index = field_count == 12U ? 10U : 9U;
  const size_t persist_index = field_count == 12U ? 11U : 10U;
  if (!ParseU32(fields_begin[1], fields_end[1], &parsed.session) ||
      !ParseU32(fields_begin[2], fields_end[2], &parsed.sequence) ||
      !ParseU32(fields_begin[3], fields_end[3], &parsed.seen_tick_ms) ||
      !ParseFlag(fields_begin[4], fields_end[4], &parsed.run) ||
      !ParseFlag(fields_begin[5], fields_end[5],
                 &parsed.temperature_enabled) ||
      !ParseU32(fields_begin[6], fields_end[6], &temperature) ||
      !ParseFlag(fields_begin[7], fields_end[7],
                 &parsed.humidity_enabled) ||
      !ParseU32(fields_begin[8], fields_end[8], &humidity) ||
      (field_count == 12U &&
       !ParseI32(fields_begin[9], fields_end[9], &ambient)) ||
      !ParseFlag(fields_begin[clear_fault_index], fields_end[clear_fault_index],
                 &parsed.clear_fault) ||
      !ParseFlag(fields_begin[persist_index], fields_end[persist_index],
                 &parsed.persist))
  {
    return false;
  }

  parsed.values_in_range = parsed.session != 0U && parsed.sequence != 0U &&
                           temperature >= 30U && temperature <= 70U &&
                           humidity >= 10U && humidity <= 80U &&
                           ambient >= REMOTE_MIN_AMBIENT_TEMPERATURE_C &&
                           ambient <= REMOTE_MAX_AMBIENT_TEMPERATURE_C;
  if (temperature <= UINT8_MAX)
  {
    parsed.target_temperature_c = (uint8_t)temperature;
  }
  if (humidity <= UINT8_MAX)
  {
    parsed.target_humidity_percent = (uint8_t)humidity;
  }
  parsed.ambient_temperature_c = (int8_t)ambient;
  *command = parsed;
  return true;
}

void RemoteControl_Init(void)
{
  parse_error_count = 0U;
  command_pending = false;
}

RemoteControlSlotResult RemoteControl_SubmitSlot(const uint8_t *slot,
                                                size_t length)
{
  if (slot == NULL || length != REMOTE_CONTROL_SLOT_BYTES)
  {
    RecordError();
    return REMOTE_CONTROL_SLOT_INVALID;
  }

  bool all_zero = true;
  bool all_one = true;
  for (size_t index = 0U; index < length; ++index)
  {
    all_zero = all_zero && slot[index] == 0U;
    all_one = all_one && slot[index] == UINT8_MAX;
  }
  if (all_zero || all_one)
  {
    /* A released/disconnected data wire reads as a uniform idle level. It is
       not a valid reply and is intentionally not counted on every probe. */
    return REMOTE_CONTROL_SLOT_INVALID;
  }

  size_t frame_length = 0U;
  while (frame_length < length && slot[frame_length] != (uint8_t)'\n')
  {
    ++frame_length;
  }

  if (frame_length == length)
  {
    RecordError();
    return REMOTE_CONTROL_SLOT_INVALID;
  }

  ++frame_length; /* Include LF in the CRC-framed command. */
  for (size_t index = frame_length; index < length; ++index)
  {
    if (slot[index] != 0U)
    {
      RecordError();
      return REMOTE_CONTROL_SLOT_INVALID;
    }
  }

  static const uint8_t nop_frame[] = "N*48FA\r\n";
  if (frame_length == sizeof(nop_frame) - 1U)
  {
    for (size_t index = 0U; index < frame_length; ++index)
    {
      if (slot[index] != nop_frame[index])
      {
        RecordError();
        return REMOTE_CONTROL_SLOT_INVALID;
      }
    }
    return REMOTE_CONTROL_SLOT_NOP;
  }

  RemoteControlCommand parsed = {0};
  if (!ParseFrame(slot, frame_length, &parsed))
  {
    RecordError();
    return REMOTE_CONTROL_SLOT_INVALID;
  }

  /* Only one command can be produced per one-second exchange. Do not replace
     an unread command if a caller violates that contract. */
  if (command_pending)
  {
    RecordError();
    return REMOTE_CONTROL_SLOT_INVALID;
  }
  pending_command = parsed;
  command_pending = true;
  return REMOTE_CONTROL_SLOT_COMMAND;
}

void RemoteControl_RecordTransportError(void)
{
  RecordError();
}

bool RemoteControl_TryRead(RemoteControlCommand *command)
{
  if (command == NULL || !command_pending)
  {
    return false;
  }

  *command = pending_command;
  command_pending = false;
  return true;
}

uint32_t RemoteControl_GetRxErrorCount(void)
{
  return parse_error_count;
}
