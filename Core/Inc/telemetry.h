#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SPI2 one-wire master: PB13=SCK, PB14=active-low frame select and
 * PB15=half-duplex data. Each cycle sends 64 bytes, releases PB15, then
 * receives one fixed 64-byte command slot.
 */
void Telemetry_Init(void);

/*
 * Sends one fixed-size CS-delimited binary frame in a bounded polling interval,
 * then receives and validates the corresponding command slot.
 * Format: "TG", version, used length, then little-endian telemetry/control
 * v3 fields through byte 39, room temperature in byte 12, zero-reserved bytes 13..15 and 40..61, and CRC16-CCITT in
 * bytes 62..63. The CRC covers bytes 0..61 and is stored little-endian.
 * Returns false on a transfer timeout or malformed command slot; the bus is
 * always released and reset so the following one-second cycle can retry.
 */
bool Telemetry_Send(uint32_t tick_ms, int32_t temperature_centi_c,
                    uint32_t humidity_milli_percent,
                    bool valid, uint8_t state_bits,
                    uint16_t output_permille, uint8_t target_temperature_c,
                    uint8_t target_humidity_percent, uint8_t fault,
                    int8_t ambient_temperature_c,
                    uint32_t ack_session, uint32_t ack_sequence,
                    uint8_t ack_result, uint32_t rx_errors);

#ifdef __cplusplus
}
#endif
#endif
