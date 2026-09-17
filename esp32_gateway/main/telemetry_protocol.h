#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One complete STM32 request occupies exactly one 64-byte SPI transaction. */
#define TELEMETRY_FRAME_MAX_BYTES 64U
#define TELEMETRY_MAGIC_0 ((uint8_t)'T')
#define TELEMETRY_MAGIC_1 ((uint8_t)'G')
#define TELEMETRY_PROTOCOL_VERSION_LEGACY 2U
#define TELEMETRY_PROTOCOL_VERSION 3U
#define TELEMETRY_CONTENT_BYTES 40U
#define TELEMETRY_CRC_OFFSET 62U
#define TELEMETRY_DEFAULT_AMBIENT_TEMPERATURE_C 22

#define TELEMETRY_STATE_RUNNING (1U << 0)
#define TELEMETRY_STATE_HEATER_ON (1U << 1)
#define TELEMETRY_STATE_MOTOR_ON (1U << 2)
#define TELEMETRY_STATE_TEMPERATURE_ENABLED (1U << 3)
#define TELEMETRY_STATE_HUMIDITY_ENABLED (1U << 4)
#define TELEMETRY_STATE_FAN_RUNNING (1U << 5)
#define TELEMETRY_STATE_FAN_VALID (1U << 6)
#define TELEMETRY_STATE_REMOTE_OWNED (1U << 7)

typedef enum {
    TELEMETRY_ACK_NONE = 0,
    TELEMETRY_ACK_APPLIED = 1,
    TELEMETRY_ACK_BAD_RANGE = 2,
    TELEMETRY_ACK_STALE = 3,
    TELEMETRY_ACK_FAULT_ACTIVE = 4,
    TELEMETRY_ACK_FAN = 5,
    TELEMETRY_ACK_SENSOR = 6,
    TELEMETRY_ACK_OVERTEMPERATURE = 7,
    TELEMETRY_ACK_CLEAR_REJECTED = 8,
    TELEMETRY_ACK_LEASE_EXPIRED = 9,
    TELEMETRY_ACK_STORAGE_FAILED = 10,
    TELEMETRY_ACK_PERSIST_REQUIRES_STOP = 11,
} telemetry_ack_result_t;

typedef struct {
    uint32_t stm32_tick_ms;
    float temperature_c;
    float humidity_percent;
    bool valid;

    /* Binary protocol v2/v3 carries the extended control status. */
    bool extended;
    uint8_t state_bits;
    uint16_t output_permille;
    uint8_t target_temperature_c;
    uint8_t target_humidity_percent;
    int8_t ambient_temperature_c;
    uint8_t fault;
    uint32_t ack_session;
    uint32_t ack_sequence;
    telemetry_ack_result_t ack_result;
    uint32_t command_rx_errors;
} telemetry_frame_t;

typedef enum {
    TELEMETRY_PARSE_OK = 0,
    TELEMETRY_PARSE_EMPTY,
    TELEMETRY_PARSE_TOO_LONG,
    TELEMETRY_PARSE_INCOMPLETE,
    TELEMETRY_PARSE_MAGIC,
    TELEMETRY_PARSE_VERSION,
    TELEMETRY_PARSE_DECLARED_LENGTH,
    TELEMETRY_PARSE_RESERVED,
    TELEMETRY_PARSE_CRC,
    TELEMETRY_PARSE_VALID_FLAG,
    TELEMETRY_PARSE_TEMPERATURE,
    TELEMETRY_PARSE_HUMIDITY,
    TELEMETRY_PARSE_OUTPUT,
    TELEMETRY_PARSE_TEMPERATURE_TARGET,
    TELEMETRY_PARSE_HUMIDITY_TARGET,
    TELEMETRY_PARSE_AMBIENT_TEMPERATURE,
    TELEMETRY_PARSE_FAULT,
    TELEMETRY_PARSE_ACK_RESULT,
} telemetry_parse_result_t;

uint16_t telemetry_crc16_ccitt_false(const uint8_t *data, size_t length);

/*
 * Parse one exact 64-byte, little-endian binary frame. Version 2 keeps bytes 12..15
 * reserved and defaults ambient_temperature_c to 22; version 3 stores ambient+10 in
 * byte 12 and keeps bytes 13..15 reserved. Bytes 40..61 must be zero. Bytes 62..63
 * contain CRC16-CCITT-FALSE over bytes 0..61, low byte first. Sensor integers use
 * 0.01 C and 0.001 %RH units.
 */
telemetry_parse_result_t telemetry_parse_frame(const uint8_t *data,
                                               size_t length,
                                               telemetry_frame_t *out);

const char *telemetry_parse_result_name(telemetry_parse_result_t result);
const char *telemetry_ack_result_name(telemetry_ack_result_t result);

#endif
