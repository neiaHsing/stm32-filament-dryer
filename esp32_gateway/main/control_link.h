#ifndef CONTROL_LINK_H
#define CONTROL_LINK_H

#include "control_protocol.h"
#include "telemetry_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define CONTROL_LINK_TELEMETRY_FRESH_MS 2500ULL

typedef struct {
    bool running;
    bool temperature_enabled;
    uint8_t target_temperature_c;
    bool humidity_enabled;
    uint8_t target_humidity_percent;
    int8_t ambient_temperature_c;
    bool clear_fault;
    bool persist;
} control_request_t;

typedef struct {
    double kp;
    double ki;
    double kd;
} control_pid_request_t;

typedef struct {
    bool has_request;
    control_pid_command_t requested;
    bool pending;
    bool applied;
    bool canceled_by_stm32_reset;
    telemetry_ack_result_t result;
} control_pid_status_t;

typedef struct {
    bool has_request;
    control_command_t requested;
    bool pending;
    bool applied;
    bool canceled_by_stm32_reset;
    telemetry_ack_result_t result;

    bool telemetry_present;
    bool telemetry_extended;
    bool telemetry_fresh;
    uint64_t telemetry_age_ms;
    uint32_t latest_stm32_tick_ms;
    bool command_sent;
    uint64_t last_send_age_ms;
    uint32_t command_tx_errors;
    control_pid_status_t pid;
} control_link_status_t;

esp_err_t control_link_init(void);

/*
 * Replaces the desired remote state and starts reliable delivery. A request
 * that starts the appliance is rejected unless fresh extended telemetry is
 * available. Stop/settings requests can still be queued while telemetry is
 * absent so a stop can never be blocked by link status.
 */
esp_err_t control_link_submit(const control_request_t *request,
                              uint32_t *session, uint32_t *sequence);

esp_err_t control_link_submit_pid(const control_pid_request_t *request,
                                  uint32_t *session, uint32_t *sequence);

/* Called once for every accepted SPI telemetry frame. */
void control_link_on_telemetry(const telemetry_frame_t *frame,
                               uint64_t received_monotonic_ms);

/*
 * Builds the next fixed-size SPI reply slot after accepted telemetry. A false
 * return value means the slot contains a valid CRC-protected NOP. At most one RUN command
 * is emitted for each accepted status frame, so a lost return path cannot be
 * hidden by locally generated lease heartbeats.
 */
bool control_link_prepare_command_slot(uint8_t *slot, size_t slot_size,
                                       uint64_t now_ms);

void control_link_note_command_tx_error(void);

void control_link_get_status(uint64_t now_ms, control_link_status_t *status);
const char *control_link_status_name(const control_link_status_t *status);

#endif
