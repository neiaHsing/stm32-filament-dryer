#include "control_protocol.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

uint16_t control_crc16_ccitt_false(const uint8_t *data, size_t length)
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

bool control_command_valid(const control_command_t *command)
{
    return command != NULL && command->session != 0U &&
           command->sequence != 0U &&
           command->target_temperature_c >= CONTROL_MIN_TEMPERATURE_C &&
           command->target_temperature_c <= CONTROL_MAX_TEMPERATURE_C &&
           command->target_humidity_percent >=
               CONTROL_MIN_HUMIDITY_PERCENT &&
           command->target_humidity_percent <=
               CONTROL_MAX_HUMIDITY_PERCENT &&
           command->ambient_temperature_c >=
               CONTROL_MIN_AMBIENT_TEMPERATURE_C &&
           command->ambient_temperature_c <=
               CONTROL_MAX_AMBIENT_TEMPERATURE_C;
}

size_t control_command_encode(const control_command_t *command, char *output,
                              size_t output_size)
{
    if (!control_command_valid(command) || output == NULL ||
        output_size < 8U) {
        return 0U;
    }

    const int body_length = snprintf(
        output, output_size,
        "C,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%u,%u,%u,%u,%u,%d,%u,%u",
        command->session, command->sequence, command->seen_stm32_tick_ms,
        command->running ? 1U : 0U,
        command->temperature_enabled ? 1U : 0U,
        (unsigned int)command->target_temperature_c,
        command->humidity_enabled ? 1U : 0U,
        (unsigned int)command->target_humidity_percent,
        (int)command->ambient_temperature_c,
        command->clear_fault ? 1U : 0U, command->persist ? 1U : 0U);
    if (body_length <= 0 || (size_t)body_length >= output_size) {
        return 0U;
    }

    const uint16_t crc = control_crc16_ccitt_false(
        (const uint8_t *)output, (size_t)body_length);
    const int suffix_length =
        snprintf(output + body_length, output_size - (size_t)body_length,
                 "*%04X\r\n", (unsigned int)crc);
    if (suffix_length != 7 ||
        (size_t)body_length + (size_t)suffix_length >= output_size) {
        return 0U;
    }
    return (size_t)body_length + (size_t)suffix_length;
}

size_t control_command_encode_slot(const control_command_t *command,
                                   uint8_t *output, size_t output_size)
{
    if (output == NULL || output_size != CONTROL_COMMAND_SLOT_BYTES) {
        return 0U;
    }

    memset(output, 0, output_size);
    const size_t length = control_command_encode(
        command, (char *)output, output_size);
    if (length == 0U) {
        memset(output, 0, output_size);
    }
    return length;
}

bool control_pid_command_valid(const control_pid_command_t *command)
{
    return command != NULL && command->session != 0U &&
           command->sequence != 0U &&
           command->kp_tenths <= CONTROL_PID_KP_MAX_TENTHS &&
           command->ki_hundredths <= CONTROL_PID_KI_MAX_HUNDREDTHS &&
           command->kd_tenths <= CONTROL_PID_KD_MAX_TENTHS;
}

size_t control_pid_command_encode_slot(const control_pid_command_t *command,
                                       uint8_t *output, size_t output_size)
{
    if (!control_pid_command_valid(command) || output == NULL ||
        output_size != CONTROL_COMMAND_SLOT_BYTES) {
        return 0U;
    }
    memset(output, 0, output_size);
    const int body_length = snprintf(
        (char *)output, output_size,
        "P,%" PRIu32 ",%" PRIu32 ",%u,%u,%u",
        command->session, command->sequence,
        (unsigned int)command->kp_tenths,
        (unsigned int)command->ki_hundredths,
        (unsigned int)command->kd_tenths);
    if (body_length <= 0 || (size_t)body_length >= output_size) {
        memset(output, 0, output_size);
        return 0U;
    }
    const uint16_t crc = control_crc16_ccitt_false(
        output, (size_t)body_length);
    const int suffix_length = snprintf(
        (char *)output + body_length, output_size - (size_t)body_length,
        "*%04X\r\n", (unsigned int)crc);
    if (suffix_length != 7 ||
        (size_t)body_length + (size_t)suffix_length >= output_size) {
        memset(output, 0, output_size);
        return 0U;
    }
    return (size_t)body_length + (size_t)suffix_length;
}

size_t control_reply_encode_nop(uint8_t *output, size_t output_size)
{
    if (output == NULL || output_size != CONTROL_COMMAND_SLOT_BYTES) {
        return 0U;
    }

    memset(output, 0, output_size);
    static const uint8_t body[] = "N";
    const uint16_t crc = control_crc16_ccitt_false(body, sizeof(body) - 1U);
    const int length = snprintf((char *)output, output_size, "N*%04X\r\n",
                                (unsigned int)crc);
    if (length != 8) {
        memset(output, 0, output_size);
        return 0U;
    }
    return (size_t)length;
}
