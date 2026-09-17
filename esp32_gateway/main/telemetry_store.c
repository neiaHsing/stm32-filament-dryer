#include "telemetry_store.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    SemaphoreHandle_t mutex;
    telemetry_sample_t *samples;
    size_t capacity;
    size_t count;
    size_t next;
    uint32_t next_sequence;
    uint32_t accepted_frames;
    uint32_t parse_errors;
    uint32_t spi_errors;
    char last_error[32];
} store_state_t;

static store_state_t s_store;

esp_err_t telemetry_store_init(size_t capacity)
{
    if (capacity == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_store, 0, sizeof(s_store));
    s_store.samples = calloc(capacity, sizeof(*s_store.samples));
    if (s_store.samples == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_store.mutex = xSemaphoreCreateMutex();
    if (s_store.mutex == NULL) {
        free(s_store.samples);
        s_store.samples = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_store.capacity = capacity;
    snprintf(s_store.last_error, sizeof(s_store.last_error), "none");
    return ESP_OK;
}

void telemetry_store_add(const telemetry_frame_t *frame, uint64_t received_ms)
{
    if (frame == NULL || s_store.mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_store.mutex, portMAX_DELAY);
    telemetry_sample_t *sample = &s_store.samples[s_store.next];
    sample->frame = *frame;
    sample->sequence = s_store.next_sequence++;
    sample->received_monotonic_ms = received_ms;
    s_store.next = (s_store.next + 1U) % s_store.capacity;
    if (s_store.count < s_store.capacity) {
        ++s_store.count;
    }
    ++s_store.accepted_frames;
    snprintf(s_store.last_error, sizeof(s_store.last_error), "none");
    xSemaphoreGive(s_store.mutex);
}

void telemetry_store_note_parse_error(telemetry_parse_result_t result)
{
    if (s_store.mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_store.mutex, portMAX_DELAY);
    ++s_store.parse_errors;
    snprintf(s_store.last_error, sizeof(s_store.last_error), "parse:%s",
             telemetry_parse_result_name(result));
    xSemaphoreGive(s_store.mutex);
}

void telemetry_store_note_spi_error(esp_err_t error)
{
    if (s_store.mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_store.mutex, portMAX_DELAY);
    ++s_store.spi_errors;
    snprintf(s_store.last_error, sizeof(s_store.last_error), "spi:0x%x",
             (unsigned int)error);
    xSemaphoreGive(s_store.mutex);
}

void telemetry_store_get_stats(telemetry_store_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    if (s_store.mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_store.mutex, portMAX_DELAY);
    stats->history_count = s_store.count;
    stats->history_capacity = s_store.capacity;
    stats->accepted_frames = s_store.accepted_frames;
    stats->parse_errors = s_store.parse_errors;
    stats->spi_errors = s_store.spi_errors;
    memcpy(stats->last_error, s_store.last_error, sizeof(stats->last_error));
    if (s_store.count > 0U) {
        const size_t newest = (s_store.next + s_store.capacity - 1U) % s_store.capacity;
        stats->current = s_store.samples[newest];
        stats->has_current = true;
    }
    xSemaphoreGive(s_store.mutex);
}

bool telemetry_store_copy_at(size_t index, telemetry_sample_t *sample)
{
    if (sample == NULL || s_store.mutex == NULL) {
        return false;
    }

    bool copied = false;
    xSemaphoreTake(s_store.mutex, portMAX_DELAY);
    if (index < s_store.count) {
        const size_t oldest = s_store.count == s_store.capacity ? s_store.next : 0U;
        *sample = s_store.samples[(oldest + index) % s_store.capacity];
        copied = true;
    }
    xSemaphoreGive(s_store.mutex);
    return copied;
}
