#include "telemetry_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static void finish_crc(uint8_t frame[TELEMETRY_FRAME_MAX_BYTES])
{
    put_u16_le(frame + TELEMETRY_CRC_OFFSET,
               telemetry_crc16_ccitt_false(frame, TELEMETRY_CRC_OFFSET));
}

static void make_valid_frame(uint8_t frame[TELEMETRY_FRAME_MAX_BYTES])
{
    memset(frame, 0, TELEMETRY_FRAME_MAX_BYTES);
    frame[0] = TELEMETRY_MAGIC_0;
    frame[1] = TELEMETRY_MAGIC_1;
    frame[2] = TELEMETRY_PROTOCOL_VERSION;
    frame[3] = TELEMETRY_CONTENT_BYTES;
    put_u32_le(frame + 4U, 0x89abcdefU);
    put_u32_le(frame + 8U, (uint32_t)-1234);
    frame[12] = 32U; /* 22 C, encoded as ambient + 10. */
    put_u32_le(frame + 16U, 51234U);
    frame[20] = 1U;
    frame[21] = 0xa5U;
    put_u16_le(frame + 22U, 1000U);
    frame[24] = 70U;
    frame[25] = 80U;
    frame[26] = 3U;
    frame[27] = TELEMETRY_ACK_PERSIST_REQUIRES_STOP;
    put_u32_le(frame + 28U, 0x10203040U);
    put_u32_le(frame + 32U, 0x50607080U);
    put_u32_le(frame + 36U, 0x90abcdefU);
    finish_crc(frame);
}

static telemetry_parse_result_t parse_changed_byte(uint8_t frame[64],
                                                    size_t offset,
                                                    uint8_t value)
{
    make_valid_frame(frame);
    frame[offset] = value;
    finish_crc(frame);
    telemetry_frame_t parsed = {0};
    return telemetry_parse_frame(frame, TELEMETRY_FRAME_MAX_BYTES, &parsed);
}

int main(void)
{
    static const uint8_t crc_vector[] = "123456789";
    assert(telemetry_crc16_ccitt_false(crc_vector,
                                      sizeof(crc_vector) - 1U) == 0x29b1U);
    assert(telemetry_crc16_ccitt_false(NULL, 0U) == 0xffffU);

    uint8_t frame[TELEMETRY_FRAME_MAX_BYTES];
    make_valid_frame(frame);
    telemetry_frame_t parsed = {0};
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_OK);
    assert(parsed.extended);
    assert(parsed.stm32_tick_ms == 0x89abcdefU);
    assert(fabsf(parsed.temperature_c - (-12.34f)) < 0.001f);
    assert(fabsf(parsed.humidity_percent - 51.234f) < 0.001f);
    assert(parsed.valid);
    assert(parsed.state_bits == 0xa5U);
    assert(parsed.output_permille == 1000U);
    assert(parsed.target_temperature_c == 70U);
    assert(parsed.target_humidity_percent == 80U);
    assert(parsed.ambient_temperature_c == 22);
    assert(parsed.fault == 3U);
    assert(parsed.ack_result == TELEMETRY_ACK_PERSIST_REQUIRES_STOP);
    assert(parsed.ack_session == 0x10203040U);
    assert(parsed.ack_sequence == 0x50607080U);
    assert(parsed.command_rx_errors == 0x90abcdefU);

    make_valid_frame(frame);
    put_u32_le(frame + 8U, 3982U);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) == TELEMETRY_PARSE_OK);
    assert(fabsf(parsed.temperature_c - 39.82f) < 0.001f);

    make_valid_frame(frame);
    frame[20] = 0U;
    put_u32_le(frame + 8U, UINT32_MAX);
    put_u32_le(frame + 16U, UINT32_MAX);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_OK);
    assert(!parsed.valid);
    assert(isnan(parsed.temperature_c));
    assert(isnan(parsed.humidity_percent));

    assert(telemetry_parse_frame(NULL, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_EMPTY);
    assert(telemetry_parse_frame(frame, sizeof(frame), NULL) ==
           TELEMETRY_PARSE_EMPTY);
    assert(telemetry_parse_frame(frame, 0U, &parsed) ==
           TELEMETRY_PARSE_EMPTY);
    assert(telemetry_parse_frame(frame, sizeof(frame) - 1U, &parsed) ==
           TELEMETRY_PARSE_INCOMPLETE);
    uint8_t oversized[TELEMETRY_FRAME_MAX_BYTES + 1U] = {0};
    assert(telemetry_parse_frame(oversized, sizeof(oversized), &parsed) ==
           TELEMETRY_PARSE_TOO_LONG);

    assert(parse_changed_byte(frame, 0U, 'X') == TELEMETRY_PARSE_MAGIC);
    assert(parse_changed_byte(frame, 1U, 'X') == TELEMETRY_PARSE_MAGIC);
    assert(parse_changed_byte(frame, 2U, 1U) == TELEMETRY_PARSE_VERSION);
    assert(parse_changed_byte(frame, 3U, 39U) ==
           TELEMETRY_PARSE_DECLARED_LENGTH);
    assert(parse_changed_byte(frame, 40U, 1U) == TELEMETRY_PARSE_RESERVED);
    assert(parse_changed_byte(frame, 61U, 1U) == TELEMETRY_PARSE_RESERVED);

    assert(parse_changed_byte(frame, 12U, 61U) ==
           TELEMETRY_PARSE_AMBIENT_TEMPERATURE);
    for (size_t i = 13U; i < 16U; ++i) {
        assert(parse_changed_byte(frame, i, 1U) == TELEMETRY_PARSE_RESERVED);
    }
    make_valid_frame(frame);
    frame[2] = TELEMETRY_PROTOCOL_VERSION_LEGACY;
    frame[12] = 0U;
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_OK);
    assert(parsed.ambient_temperature_c == 22);
    make_valid_frame(frame);
    put_u32_le(frame + 8U, 8501U);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) == TELEMETRY_PARSE_TEMPERATURE);

    make_valid_frame(frame);
    frame[10] ^= 1U;
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_CRC);

    assert(parse_changed_byte(frame, 20U, 2U) ==
           TELEMETRY_PARSE_VALID_FLAG);

    make_valid_frame(frame);
    put_u32_le(frame + 8U, (uint32_t)-4001);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_TEMPERATURE);

    make_valid_frame(frame);
    put_u32_le(frame + 12U, 8501U);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_RESERVED);

    make_valid_frame(frame);
    put_u32_le(frame + 16U, 100001U);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_HUMIDITY);

    make_valid_frame(frame);
    put_u16_le(frame + 22U, 1001U);
    finish_crc(frame);
    assert(telemetry_parse_frame(frame, sizeof(frame), &parsed) ==
           TELEMETRY_PARSE_OUTPUT);

    assert(parse_changed_byte(frame, 24U, 29U) ==
           TELEMETRY_PARSE_TEMPERATURE_TARGET);
    assert(parse_changed_byte(frame, 24U, 71U) ==
           TELEMETRY_PARSE_TEMPERATURE_TARGET);
    assert(parse_changed_byte(frame, 25U, 9U) ==
           TELEMETRY_PARSE_HUMIDITY_TARGET);
    assert(parse_changed_byte(frame, 25U, 81U) ==
           TELEMETRY_PARSE_HUMIDITY_TARGET);
    assert(parse_changed_byte(frame, 26U, 4U) == TELEMETRY_PARSE_FAULT);
    assert(parse_changed_byte(frame, 27U, 12U) ==
           TELEMETRY_PARSE_ACK_RESULT);

    assert(strcmp(telemetry_parse_result_name(TELEMETRY_PARSE_CRC),
                  "crc") == 0);
    puts("telemetry protocol tests passed");
    return 0;
}
