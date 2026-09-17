#include "telemetry_protocol.h"

#include <limits.h>
#include <math.h>

#define TEMPERATURE_MIN_CENTI_C (-4000)
#define TEMPERATURE_MAX_CENTI_C 8500
#define HUMIDITY_MAX_MILLI_PERCENT 100000U
#define OUTPUT_MAX_PERMILLE 1000U
#define TEMPERATURE_TARGET_MIN_C 30U
#define TEMPERATURE_TARGET_MAX_C 70U
#define HUMIDITY_TARGET_MIN_PERCENT 10U
#define HUMIDITY_TARGET_MAX_PERCENT 80U
#define FAULT_MAX 3U

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static int32_t read_i32_le(const uint8_t *data)
{
    const uint32_t raw = read_u32_le(data);
    if (raw <= INT32_MAX) {
        return (int32_t)raw;
    }
    /* Decode two's-complement without relying on an out-of-range cast. */
    return -1 - (int32_t)(UINT32_MAX - raw);
}

uint16_t telemetry_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffffU;
    if (data == NULL && length != 0U) {
        return crc;
    }

    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? (uint16_t)((crc << 1U) ^ 0x1021U)
                      : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

telemetry_parse_result_t telemetry_parse_frame(const uint8_t *data,
                                               size_t length,
                                               telemetry_frame_t *out)
{
    if (data == NULL || out == NULL || length == 0U) {
        return TELEMETRY_PARSE_EMPTY;
    }
    if (length < TELEMETRY_FRAME_MAX_BYTES) {
        return TELEMETRY_PARSE_INCOMPLETE;
    }
    if (length > TELEMETRY_FRAME_MAX_BYTES) {
        return TELEMETRY_PARSE_TOO_LONG;
    }
    if (data[0] != TELEMETRY_MAGIC_0 || data[1] != TELEMETRY_MAGIC_1) {
        return TELEMETRY_PARSE_MAGIC;
    }
    if (data[2] != TELEMETRY_PROTOCOL_VERSION &&
        data[2] != TELEMETRY_PROTOCOL_VERSION_LEGACY) {
        return TELEMETRY_PARSE_VERSION;
    }
    if (data[3] != TELEMETRY_CONTENT_BYTES) {
        return TELEMETRY_PARSE_DECLARED_LENGTH;
    }
    for (size_t index = TELEMETRY_CONTENT_BYTES;
         index < TELEMETRY_CRC_OFFSET; ++index) {
        if (data[index] != 0U) {
            return TELEMETRY_PARSE_RESERVED;
        }
    }
    const bool has_ambient = data[2] == TELEMETRY_PROTOCOL_VERSION;
    for (size_t index = has_ambient ? 13U : 12U; index < 16U; ++index) {
        if (data[index] != 0U) return TELEMETRY_PARSE_RESERVED;
    }
    if (read_u16_le(data + TELEMETRY_CRC_OFFSET) !=
        telemetry_crc16_ccitt_false(data, TELEMETRY_CRC_OFFSET)) {
        return TELEMETRY_PARSE_CRC;
    }

    const uint8_t valid = data[20];
    if (valid > 1U) {
        return TELEMETRY_PARSE_VALID_FLAG;
    }

    const int32_t temperature_centi_c = read_i32_le(data + 8U);
    const uint32_t humidity_milli_percent = read_u32_le(data + 16U);
    if (valid != 0U) {
        if (temperature_centi_c < TEMPERATURE_MIN_CENTI_C ||
            temperature_centi_c > TEMPERATURE_MAX_CENTI_C) {
            return TELEMETRY_PARSE_TEMPERATURE;
        }
        if (humidity_milli_percent > HUMIDITY_MAX_MILLI_PERCENT) {
            return TELEMETRY_PARSE_HUMIDITY;
        }
    }

    const uint16_t output_permille = read_u16_le(data + 22U);
    if (output_permille > OUTPUT_MAX_PERMILLE) {
        return TELEMETRY_PARSE_OUTPUT;
    }
    if (data[24] < TEMPERATURE_TARGET_MIN_C ||
        data[24] > TEMPERATURE_TARGET_MAX_C) {
        return TELEMETRY_PARSE_TEMPERATURE_TARGET;
    }
    if (data[25] < HUMIDITY_TARGET_MIN_PERCENT ||
        data[25] > HUMIDITY_TARGET_MAX_PERCENT) {
        return TELEMETRY_PARSE_HUMIDITY_TARGET;
    }
    if (has_ambient && data[12] >
                       (uint8_t)(50 - (-10))) {
        return TELEMETRY_PARSE_AMBIENT_TEMPERATURE;
    }
    if (data[26] > FAULT_MAX) {
        return TELEMETRY_PARSE_FAULT;
    }
    if (data[27] > TELEMETRY_ACK_PERSIST_REQUIRES_STOP) {
        return TELEMETRY_PARSE_ACK_RESULT;
    }

    telemetry_frame_t parsed = {
        .stm32_tick_ms = read_u32_le(data + 4U),
        .valid = valid != 0U,
        .extended = true,
        .state_bits = data[21],
        .output_permille = output_permille,
        .target_temperature_c = data[24],
        .target_humidity_percent = data[25],
        .ambient_temperature_c = has_ambient
                                     ? (int8_t)((int16_t)data[12] - 10)
                                     : TELEMETRY_DEFAULT_AMBIENT_TEMPERATURE_C,
        .fault = data[26],
        .ack_session = read_u32_le(data + 28U),
        .ack_sequence = read_u32_le(data + 32U),
        .ack_result = (telemetry_ack_result_t)data[27],
        .command_rx_errors = read_u32_le(data + 36U),
    };
    if (parsed.valid) {
        parsed.temperature_c = (float)temperature_centi_c / 100.0f;
        parsed.humidity_percent =
            (float)humidity_milli_percent / 1000.0f;
    } else {
        parsed.temperature_c = NAN;
        parsed.humidity_percent = NAN;
    }

    *out = parsed;
    return TELEMETRY_PARSE_OK;
}

const char *telemetry_parse_result_name(telemetry_parse_result_t result)
{
    static const char *const names[] = {
        "ok",          "empty",       "too_long", "incomplete",
        "magic",       "version",     "length",   "reserved",
        "crc",         "valid_flag",  "temperature",             "humidity", "output",
        "temperature_target", "humidity_target", "ambient_temperature",
        "fault",       "ack_result",
    };
    const size_t count = sizeof(names) / sizeof(names[0]);
    return (size_t)result < count ? names[result] : "unknown";
}

const char *telemetry_ack_result_name(telemetry_ack_result_t result)
{
    static const char *const names[] = {
        "none",          "applied",         "bad_range",
        "stale",         "fault_active",    "fan",
        "sensor",        "overtemperature", "clear_rejected",
        "lease_expired", "storage_failed", "persist_requires_stop",
    };
    const size_t count = sizeof(names) / sizeof(names[0]);
    return (size_t)result < count ? names[result] : "unknown";
}
