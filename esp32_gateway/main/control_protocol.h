#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CONTROL_COMMAND_MAX_BYTES 96U
#define CONTROL_COMMAND_SLOT_BYTES 64U
#define CONTROL_MIN_TEMPERATURE_C 30U
#define CONTROL_MAX_TEMPERATURE_C 70U
#define CONTROL_MIN_HUMIDITY_PERCENT 10U
#define CONTROL_MAX_HUMIDITY_PERCENT 80U
#define CONTROL_MIN_AMBIENT_TEMPERATURE_C (-10)
#define CONTROL_MAX_AMBIENT_TEMPERATURE_C 50
#define CONTROL_DEFAULT_AMBIENT_TEMPERATURE_C 22
#define CONTROL_PID_KP_MAX_TENTHS 5000U
#define CONTROL_PID_KI_MAX_HUNDREDTHS 5000U
#define CONTROL_PID_KD_MAX_TENTHS 10000U

typedef struct {
    uint32_t session;
    uint32_t sequence;
    uint32_t seen_stm32_tick_ms;
    bool running;
    bool temperature_enabled;
    uint8_t target_temperature_c;
    bool humidity_enabled;
    uint8_t target_humidity_percent;
    int8_t ambient_temperature_c;
    bool clear_fault;
    bool persist;
} control_command_t;

typedef struct {
    uint32_t session;
    uint32_t sequence;
    uint16_t kp_tenths;
    uint16_t ki_hundredths;
    uint16_t kd_tenths;
} control_pid_command_t;

uint16_t control_crc16_ccitt_false(const uint8_t *data, size_t length);
bool control_command_valid(const control_command_t *command);

/*
 * Encodes a command and its CRC as one CRLF-terminated transport line.
 * Returns the number of bytes to transmit, or zero when invalid/too small.
 */
size_t control_command_encode(const control_command_t *command, char *output,
                              size_t output_size);

/*
 * Encodes one fixed-size half-duplex SPI reply slot. Bytes after the CRLF are
 * zero. A zero return value indicates an invalid command or buffer.
 */
size_t control_command_encode_slot(const control_command_t *command,
                                   uint8_t *output, size_t output_size);

bool control_pid_command_valid(const control_pid_command_t *command);
size_t control_pid_command_encode_slot(const control_pid_command_t *command,
                                       uint8_t *output, size_t output_size);

/* Encodes the CRC-protected `N` reply used when no command is pending. */
size_t control_reply_encode_nop(uint8_t *output, size_t output_size);

#endif
