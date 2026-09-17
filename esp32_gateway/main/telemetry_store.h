#ifndef TELEMETRY_STORE_H
#define TELEMETRY_STORE_H

#include "telemetry_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    telemetry_frame_t frame;
    uint32_t sequence;
    uint64_t received_monotonic_ms;
} telemetry_sample_t;

typedef struct {
    bool has_current;
    telemetry_sample_t current;
    size_t history_count;
    size_t history_capacity;
    uint32_t accepted_frames;
    uint32_t parse_errors;
    uint32_t spi_errors;
    char last_error[32];
} telemetry_store_stats_t;

esp_err_t telemetry_store_init(size_t capacity);
void telemetry_store_add(const telemetry_frame_t *frame, uint64_t received_ms);
void telemetry_store_note_parse_error(telemetry_parse_result_t result);
void telemetry_store_note_spi_error(esp_err_t error);
void telemetry_store_get_stats(telemetry_store_stats_t *stats);

/* index is chronological: zero is the oldest retained sample. */
bool telemetry_store_copy_at(size_t index, telemetry_sample_t *sample);

#endif
