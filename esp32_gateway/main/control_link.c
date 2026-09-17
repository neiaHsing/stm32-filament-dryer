#include "control_link.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    SemaphoreHandle_t mutex;
    uint32_t session;
    uint32_t next_sequence;

    bool has_telemetry;
    bool telemetry_extended;
    uint32_t latest_stm32_tick_ms;
    uint64_t telemetry_received_ms;
    uint32_t telemetry_generation;

    bool has_request;
    control_command_t command;
    bool transmit_enabled;
    bool ack_received;
    bool applied;
    bool canceled_by_reset;
    telemetry_ack_result_t result;
    uint32_t send_count;
    uint64_t last_send_ms;
    uint32_t last_send_generation;
    bool pid_has_request;
    control_pid_command_t pid_command;
    bool pid_transmit_enabled;
    bool pid_ack_received;
    bool pid_applied;
    bool pid_canceled_by_reset;
    telemetry_ack_result_t pid_result;
    uint32_t pid_last_send_generation;
    uint32_t command_tx_errors;
} control_state_t;

static const char *TAG = "control_link";
static control_state_t s_control;

static bool tick_rolled_back(uint32_t previous, uint32_t current)
{
    return current < previous && (previous - current) < (UINT32_MAX / 2U);
}

static uint64_t monotonic_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000LL);
}

static uint32_t next_sequence(void)
{
    ++s_control.next_sequence;
    if (s_control.next_sequence == 0U) {
        ++s_control.next_sequence;
    }
    return s_control.next_sequence;
}

esp_err_t control_link_init(void)
{
    memset(&s_control, 0, sizeof(s_control));
    s_control.mutex = xSemaphoreCreateMutex();
    if (s_control.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    do {
        s_control.session = esp_random();
    } while (s_control.session == 0U);
    ESP_LOGI(TAG, "Control replies use the GPIO23 half-duplex SPI slot");
    return ESP_OK;
}

esp_err_t control_link_submit(const control_request_t *request,
                              uint32_t *session, uint32_t *sequence)
{
    const bool requested_running =
        request != NULL && request->running && !request->clear_fault;
    if (request == NULL ||
        request->target_temperature_c < CONTROL_MIN_TEMPERATURE_C ||
        request->target_temperature_c > CONTROL_MAX_TEMPERATURE_C ||
        request->target_humidity_percent < CONTROL_MIN_HUMIDITY_PERCENT ||
        request->target_humidity_percent > CONTROL_MAX_HUMIDITY_PERCENT ||
        request->ambient_temperature_c < CONTROL_MIN_AMBIENT_TEMPERATURE_C ||
        request->ambient_temperature_c > CONTROL_MAX_AMBIENT_TEMPERATURE_C ||
        (requested_running && request->persist)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_control.mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint64_t now_ms = monotonic_ms();
    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    const uint64_t telemetry_age =
        s_control.has_telemetry && now_ms >= s_control.telemetry_received_ms
            ? now_ms - s_control.telemetry_received_ms
            : UINT64_MAX;
    if (requested_running &&
        (!s_control.has_telemetry || !s_control.telemetry_extended ||
         telemetry_age > CONTROL_LINK_TELEMETRY_FRESH_MS)) {
        xSemaphoreGive(s_control.mutex);
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t sequence_number = next_sequence();
    s_control.command = (control_command_t){
        .session = s_control.session,
        .sequence = sequence_number,
        .seen_stm32_tick_ms =
            s_control.has_telemetry ? s_control.latest_stm32_tick_ms : 0U,
        /* Fault clearing is always a stopped-state operation. */
        .running = requested_running,
        .temperature_enabled = request->temperature_enabled,
        .target_temperature_c = request->target_temperature_c,
        .humidity_enabled = request->humidity_enabled,
        .target_humidity_percent = request->target_humidity_percent,
        .ambient_temperature_c = request->ambient_temperature_c,
        .clear_fault = request->clear_fault,
        .persist = request->persist,
    };
    s_control.has_request = true;
    s_control.transmit_enabled = true;
    s_control.ack_received = false;
    s_control.applied = false;
    s_control.canceled_by_reset = false;
    s_control.result = TELEMETRY_ACK_NONE;
    s_control.send_count = 0U;
    s_control.last_send_ms = 0U;
    s_control.last_send_generation = 0U;
    if (session != NULL) {
        *session = s_control.session;
    }
    if (sequence != NULL) {
        *sequence = s_control.command.sequence;
    }
    xSemaphoreGive(s_control.mutex);
    return ESP_OK;
}

esp_err_t control_link_submit_pid(const control_pid_request_t *request,
                                  uint32_t *session, uint32_t *sequence)
{
    if (request == NULL || !isfinite(request->kp) ||
        !isfinite(request->ki) || !isfinite(request->kd) ||
        request->kp < 0.0 || request->kp > CONTROL_PID_KP_MAX_TENTHS / 10.0 ||
        request->ki < 0.0 || request->ki > CONTROL_PID_KI_MAX_HUNDREDTHS / 100.0 ||
        request->kd < 0.0 || request->kd > CONTROL_PID_KD_MAX_TENTHS / 10.0) {
        return ESP_ERR_INVALID_ARG;
    }
    const double kp_scaled = request->kp * 10.0;
    const double ki_scaled = request->ki * 100.0;
    const double kd_scaled = request->kd * 10.0;
    if (floor(kp_scaled) != kp_scaled || floor(ki_scaled) != ki_scaled ||
        floor(kd_scaled) != kd_scaled || s_control.mutex == NULL) {
        return s_control.mutex == NULL ? ESP_ERR_INVALID_STATE
                                       : ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    const uint32_t sequence_number = next_sequence();
    s_control.pid_command = (control_pid_command_t){
        .session = s_control.session,
        .sequence = sequence_number,
        .kp_tenths = (uint16_t)kp_scaled,
        .ki_hundredths = (uint16_t)ki_scaled,
        .kd_tenths = (uint16_t)kd_scaled,
    };
    s_control.pid_has_request = true;
    s_control.pid_transmit_enabled = true;
    s_control.pid_ack_received = false;
    s_control.pid_applied = false;
    s_control.pid_canceled_by_reset = false;
    s_control.pid_result = TELEMETRY_ACK_NONE;
    s_control.pid_last_send_generation = 0U;
    if (session != NULL) *session = s_control.session;
    if (sequence != NULL) *sequence = sequence_number;
    xSemaphoreGive(s_control.mutex);
    return ESP_OK;
}

void control_link_on_telemetry(const telemetry_frame_t *frame,
                               uint64_t received_monotonic_ms)
{
    if (frame == NULL || s_control.mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    const bool reset =
        s_control.has_telemetry &&
        tick_rolled_back(s_control.latest_stm32_tick_ms,
                         frame->stm32_tick_ms);
    s_control.has_telemetry = true;
    s_control.telemetry_extended = frame->extended;
    s_control.latest_stm32_tick_ms = frame->stm32_tick_ms;
    s_control.telemetry_received_ms = received_monotonic_ms;
    ++s_control.telemetry_generation;
    if (s_control.telemetry_generation == 0U) {
        ++s_control.telemetry_generation;
    }

    if (reset && s_control.has_request && s_control.command.running) {
        /*
         * Never replay a start that predates a controller reset. The caller
         * must make a fresh request using the new STM32 tick.
         */
        s_control.transmit_enabled = false;
        s_control.ack_received = false;
        s_control.applied = false;
        s_control.canceled_by_reset = true;
        s_control.result = TELEMETRY_ACK_NONE;
        ESP_LOGW(TAG, "STM32 tick rollback; canceled remote start");
    }

    if (reset && s_control.pid_has_request && s_control.pid_transmit_enabled) {
        s_control.pid_transmit_enabled = false;
        s_control.pid_ack_received = false;
        s_control.pid_applied = false;
        s_control.pid_canceled_by_reset = true;
        s_control.pid_result = TELEMETRY_ACK_NONE;
        ESP_LOGW(TAG, "STM32 tick rollback; canceled PID update");
    }

    if (!s_control.pid_canceled_by_reset && frame->extended &&
        s_control.pid_has_request &&
        frame->ack_session == s_control.pid_command.session &&
        frame->ack_sequence == s_control.pid_command.sequence &&
        frame->ack_result != TELEMETRY_ACK_NONE) {
        s_control.pid_ack_received = true;
        s_control.pid_result = frame->ack_result;
        s_control.pid_applied = frame->ack_result == TELEMETRY_ACK_APPLIED;
        s_control.pid_transmit_enabled = false;
    }

    if (!s_control.canceled_by_reset && frame->extended &&
        s_control.has_request &&
        frame->ack_session == s_control.command.session &&
        frame->ack_sequence == s_control.command.sequence &&
        frame->ack_result != TELEMETRY_ACK_NONE) {
        s_control.ack_received = true;
        s_control.result = frame->ack_result;
        s_control.applied = frame->ack_result == TELEMETRY_ACK_APPLIED;
        /* Storage failure does not undo an already applied RUN request. */
        s_control.transmit_enabled =
            s_control.command.running &&
            (frame->ack_result == TELEMETRY_ACK_APPLIED ||
             frame->ack_result == TELEMETRY_ACK_STORAGE_FAILED);
    }
    xSemaphoreGive(s_control.mutex);
}

bool control_link_prepare_command_slot(uint8_t *slot, size_t slot_size,
                                       uint64_t now_ms)
{
    if (slot == NULL || slot_size != CONTROL_COMMAND_SLOT_BYTES) {
        return false;
    }
    if (control_reply_encode_nop(slot, slot_size) == 0U) {
        return false;
    }
    if (s_control.mutex == NULL) {
        return false;
    }

    bool command_ready = false;
    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    if (s_control.pid_has_request && s_control.pid_transmit_enabled &&
        s_control.telemetry_generation != 0U &&
        s_control.pid_last_send_generation != s_control.telemetry_generation) {
        const size_t length = control_pid_command_encode_slot(
            &s_control.pid_command, slot, slot_size);
        if (length > 0U) {
            s_control.pid_last_send_generation = s_control.telemetry_generation;
            ++s_control.send_count;
            s_control.last_send_ms = now_ms;
            command_ready = true;
        } else {
            (void)control_reply_encode_nop(slot, slot_size);
            ++s_control.command_tx_errors;
        }
    } else if (s_control.has_request && s_control.transmit_enabled &&
        s_control.telemetry_generation != 0U &&
        s_control.last_send_generation != s_control.telemetry_generation) {
        const bool telemetry_fresh =
            s_control.has_telemetry && s_control.telemetry_extended &&
            now_ms >= s_control.telemetry_received_ms &&
            now_ms - s_control.telemetry_received_ms <=
                CONTROL_LINK_TELEMETRY_FRESH_MS;
        const bool send_allowed =
            !s_control.command.running || telemetry_fresh;
        if (send_allowed) {
            const size_t length = control_command_encode_slot(
                &s_control.command, slot, slot_size);
            if (length > 0U) {
                s_control.last_send_ms = now_ms;
                s_control.last_send_generation =
                    s_control.telemetry_generation;
                ++s_control.send_count;
                command_ready = true;
            } else {
                (void)control_reply_encode_nop(slot, slot_size);
                ++s_control.command_tx_errors;
                ESP_LOGE(TAG, "Could not encode SPI command slot");
            }
        }
    }
    xSemaphoreGive(s_control.mutex);
    return command_ready;
}

void control_link_note_command_tx_error(void)
{
    if (s_control.mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    ++s_control.command_tx_errors;
    xSemaphoreGive(s_control.mutex);
}

void control_link_get_status(uint64_t now_ms, control_link_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->telemetry_age_ms = UINT64_MAX;
    status->last_send_age_ms = UINT64_MAX;
    if (s_control.mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_control.mutex, portMAX_DELAY);
    status->has_request = s_control.has_request;
    status->requested = s_control.command;
    status->pending =
        s_control.has_request && !s_control.ack_received &&
        !s_control.canceled_by_reset;
    status->applied = s_control.applied;
    status->canceled_by_stm32_reset = s_control.canceled_by_reset;
    status->result = s_control.result;
    status->telemetry_present = s_control.has_telemetry;
    status->telemetry_extended = s_control.telemetry_extended;
    status->latest_stm32_tick_ms = s_control.latest_stm32_tick_ms;
    if (s_control.has_telemetry &&
        now_ms >= s_control.telemetry_received_ms) {
        status->telemetry_age_ms =
            now_ms - s_control.telemetry_received_ms;
        status->telemetry_fresh =
            status->telemetry_age_ms <= CONTROL_LINK_TELEMETRY_FRESH_MS;
    }
    status->command_sent = s_control.send_count > 0U;
    if (status->command_sent && now_ms >= s_control.last_send_ms) {
        status->last_send_age_ms = now_ms - s_control.last_send_ms;
    }
    status->command_tx_errors = s_control.command_tx_errors;
    status->pid.has_request = s_control.pid_has_request;
    status->pid.requested = s_control.pid_command;
    status->pid.pending = s_control.pid_has_request &&
                          !s_control.pid_ack_received &&
                          !s_control.pid_canceled_by_reset;
    status->pid.applied = s_control.pid_applied;
    status->pid.canceled_by_stm32_reset = s_control.pid_canceled_by_reset;
    status->pid.result = s_control.pid_result;
    xSemaphoreGive(s_control.mutex);
}

const char *control_link_status_name(const control_link_status_t *status)
{
    if (status == NULL || !status->has_request) {
        return "idle";
    }
    if (status->canceled_by_stm32_reset) {
        return "stm32_reset";
    }
    if (status->pending) {
        return "pending";
    }
    if (status->applied) {
        return "applied";
    }
    return status->result == TELEMETRY_ACK_NONE ? "not_sent" : "rejected";
}
