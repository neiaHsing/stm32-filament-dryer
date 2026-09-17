#include "control_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    static const uint8_t check[] = "123456789";
    assert(control_crc16_ccitt_false(check, sizeof(check) - 1U) == 0x29B1U);

    control_command_t command = {
        .session = 305419896U,
        .sequence = 42U,
        .seen_stm32_tick_ms = 123456U,
        .running = true,
        .temperature_enabled = true,
        .target_temperature_c = 55U,
        .humidity_enabled = true,
        .target_humidity_percent = 20U,
        .ambient_temperature_c = 22,
        .clear_fault = false,
        .persist = false,
    };
    char encoded[CONTROL_COMMAND_MAX_BYTES];
    const size_t length = control_command_encode(&command, encoded,
                                                  sizeof(encoded));
    static const char expected[] =
        "C,305419896,42,123456,1,1,55,1,20,22,0,0*70BA\r\n";
    assert(length == sizeof(expected) - 1U);
    assert(memcmp(encoded, expected, length) == 0);

    uint8_t slot[CONTROL_COMMAND_SLOT_BYTES];
    memset(slot, 0xa5, sizeof(slot));
    const size_t nop_length = control_reply_encode_nop(slot, sizeof(slot));
    static const char expected_nop[] = "N*48FA\r\n";
    assert(nop_length == sizeof(expected_nop) - 1U);
    assert(memcmp(slot, expected_nop, nop_length) == 0);
    for (size_t index = nop_length; index < sizeof(slot); ++index) {
        assert(slot[index] == 0U);
    }

    memset(slot, 0xa5, sizeof(slot));
    assert(control_command_encode_slot(&command, slot, sizeof(slot)) ==
           length);
    assert(memcmp(slot, expected, length) == 0);
    for (size_t index = length; index < sizeof(slot); ++index) {
        assert(slot[index] == 0U);
    }

    control_command_t maximum = {
        .session = UINT32_MAX,
        .sequence = UINT32_MAX,
        .seen_stm32_tick_ms = UINT32_MAX,
        .running = true,
        .temperature_enabled = true,
        .target_temperature_c = CONTROL_MAX_TEMPERATURE_C,
        .humidity_enabled = true,
        .target_humidity_percent = CONTROL_MAX_HUMIDITY_PERCENT,
        .clear_fault = true,
        .persist = true,
    };
    memset(slot, 0xa5, sizeof(slot));
    const size_t maximum_length =
        control_command_encode_slot(&maximum, slot, sizeof(slot));
    assert(maximum_length > 0U);
    assert(maximum_length < CONTROL_COMMAND_SLOT_BYTES);
    assert(slot[maximum_length - 2U] == '\r');
    assert(slot[maximum_length - 1U] == '\n');
    assert(slot[maximum_length] == 0U);

    uint8_t wrong_size[CONTROL_COMMAND_SLOT_BYTES - 1U];
    assert(control_command_encode_slot(&command, wrong_size,
                                       sizeof(wrong_size)) == 0U);

    char too_small[8];
    assert(control_command_encode(&command, too_small, sizeof(too_small)) == 0U);
    command.session = 0U;
    assert(!control_command_valid(&command));
    memset(slot, 0xa5, sizeof(slot));
    assert(control_command_encode_slot(&command, slot, sizeof(slot)) == 0U);
    for (size_t index = 0U; index < sizeof(slot); ++index) {
        assert(slot[index] == 0U);
    }
    command.session = 1U;
    command.sequence = 0U;
    assert(!control_command_valid(&command));
    command.sequence = 1U;
    command.target_temperature_c = 29U;
    assert(!control_command_valid(&command));
    command.target_temperature_c = 55U;
    command.target_humidity_percent = 81U;
    assert(!control_command_valid(&command));
    command.target_humidity_percent = 20U;
    command.ambient_temperature_c = 51;
    assert(!control_command_valid(&command));
    command.ambient_temperature_c = 22;

    control_pid_command_t pid = {
        .session = 305419896U,
        .sequence = 43U,
        .kp_tenths = 1600U,
        .ki_hundredths = 200U,
        .kd_tenths = 1800U,
    };
    assert(control_pid_command_valid(&pid));
    memset(slot, 0xa5, sizeof(slot));
    const size_t pid_length =
        control_pid_command_encode_slot(&pid, slot, sizeof(slot));
    assert(pid_length > 0U && pid_length < sizeof(slot));
    assert(slot[0] == 'P');
    assert(slot[pid_length - 2U] == '\r');
    assert(slot[pid_length - 1U] == '\n');
    for (size_t index = pid_length; index < sizeof(slot); ++index) {
        assert(slot[index] == 0U);
    }
    pid.kd_tenths = CONTROL_PID_KD_MAX_TENTHS + 1U;
    assert(!control_pid_command_valid(&pid));

    puts("control protocol tests passed");
    return 0;
}
