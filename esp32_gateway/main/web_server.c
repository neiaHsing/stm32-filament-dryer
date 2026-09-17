#include "web_server.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "cJSON.h"
#include "control_link.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "index_html.h"
#include "wifi_manager.h"

#define STATUS_STALE_AFTER_MS 2500ULL
#define MAX_HISTORY_RESPONSE_SAMPLES 900U
#define MAX_WS_CLIENTS 8U

static const char *TAG = "web_server";
static httpd_handle_t s_server;

typedef struct {
    char json[640];
} websocket_message_t;

static const char *json_bool(bool value)
{
    return value ? "true" : "false";
}

static const char *fault_name(uint8_t fault)
{
    static const char *const names[] = {
        "none", "fan", "sensor", "overtemperature",
    };
    return fault < sizeof(names) / sizeof(names[0]) ? names[fault] : "unknown";
}

static int sample_json(char *buffer, size_t buffer_size,
                       const telemetry_sample_t *sample)
{
    char sensor_fields[112];
    if (sample->frame.valid) {
        snprintf(sensor_fields, sizeof(sensor_fields),
                 "\"temperature_c\":%.3f,\"humidity_pct\":%.3f,"
                 "\"valid\":true",
                 sample->frame.temperature_c,
                 sample->frame.humidity_percent);
    } else {
        snprintf(sensor_fields, sizeof(sensor_fields),
                 "\"temperature_c\":null,\"humidity_pct\":null,"
                 "\"valid\":false");
    }

    if (!sample->frame.extended) {
        return snprintf(
            buffer, buffer_size,
            "{\"type\":\"sample\",\"sequence\":%" PRIu32
            ",\"received_monotonic_ms\":%" PRIu64
            ",\"stm32_tick_ms\":%" PRIu32 ",%s,\"extended\":false}",
            sample->sequence, sample->received_monotonic_ms,
            sample->frame.stm32_tick_ms, sensor_fields);
    }

    const uint8_t state = sample->frame.state_bits;
    return snprintf(
        buffer, buffer_size,
        "{\"type\":\"sample\",\"sequence\":%" PRIu32
        ",\"received_monotonic_ms\":%" PRIu64
        ",\"stm32_tick_ms\":%" PRIu32 ",%s,\"extended\":true"
        ",\"state_bits\":%u,\"running\":%s,\"heater_on\":%s,\"motor_on\":%s"
        ",\"temperature_enabled\":%s,\"humidity_enabled\":%s"
        ",\"fan_running\":%s,\"fan_valid\":%s,\"remote_owned\":%s"
        ",\"output_permille\":%u,\"target_temperature_c\":%u"
        ",\"target_humidity_pct\":%u,\"ambient_temperature_c\":%d"
        ",\"fault\":%u,\"fault_name\":\"%s\""
        ",\"ack_session\":%" PRIu32 ",\"ack_sequence\":%" PRIu32
        ",\"ack_result\":%u,\"ack_result_name\":\"%s\""
        ",\"command_rx_errors\":%" PRIu32 "}",
        sample->sequence, sample->received_monotonic_ms,
        sample->frame.stm32_tick_ms, sensor_fields,
        (unsigned int)state,
        json_bool((state & TELEMETRY_STATE_RUNNING) != 0U),
        json_bool((state & TELEMETRY_STATE_HEATER_ON) != 0U),
        json_bool((state & TELEMETRY_STATE_MOTOR_ON) != 0U),
        json_bool((state & TELEMETRY_STATE_TEMPERATURE_ENABLED) != 0U),
        json_bool((state & TELEMETRY_STATE_HUMIDITY_ENABLED) != 0U),
        json_bool((state & TELEMETRY_STATE_FAN_RUNNING) != 0U),
        json_bool((state & TELEMETRY_STATE_FAN_VALID) != 0U),
        json_bool((state & TELEMETRY_STATE_REMOTE_OWNED) != 0U),
        (unsigned int)sample->frame.output_permille,
        (unsigned int)sample->frame.target_temperature_c,
        (unsigned int)sample->frame.target_humidity_percent,
        (int)sample->frame.ambient_temperature_c,
        (unsigned int)sample->frame.fault,
        fault_name(sample->frame.fault), sample->frame.ack_session,
        sample->frame.ack_sequence,
        (unsigned int)sample->frame.ack_result,
        telemetry_ack_result_name(sample->frame.ack_result),
        sample->frame.command_rx_errors);
}

static esp_err_t send_json(httpd_req_t *request, const char *json)
{
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, json);
}

static esp_err_t index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, (const char *)s_index_html,
                           s_index_html_len);
}

static void add_sample_to_object(cJSON *object,
                                 const telemetry_sample_t *sample,
                                 uint64_t now_ms)
{
    cJSON_AddNumberToObject(object, "sequence", sample->sequence);
    cJSON_AddNumberToObject(object, "received_monotonic_ms",
                            (double)sample->received_monotonic_ms);
    cJSON_AddNumberToObject(object, "stm32_tick_ms", sample->frame.stm32_tick_ms);
    cJSON_AddBoolToObject(object, "valid", sample->frame.valid);
    const uint64_t age = now_ms >= sample->received_monotonic_ms
                             ? now_ms - sample->received_monotonic_ms
                             : 0U;
    cJSON_AddNumberToObject(object, "age_ms", (double)age);
    cJSON_AddBoolToObject(object, "stale", age > STATUS_STALE_AFTER_MS);
    if (sample->frame.valid) {
        cJSON_AddNumberToObject(object, "temperature_c", sample->frame.temperature_c);
        cJSON_AddNumberToObject(object, "humidity_pct",
                                sample->frame.humidity_percent);
    } else {
        cJSON_AddNullToObject(object, "temperature_c");
        cJSON_AddNullToObject(object, "humidity_pct");
    }

    cJSON_AddBoolToObject(object, "extended", sample->frame.extended);
    if (sample->frame.extended) {
        const uint8_t state = sample->frame.state_bits;
        cJSON_AddNumberToObject(object, "state_bits", state);
        cJSON_AddBoolToObject(object, "running",
                              (state & TELEMETRY_STATE_RUNNING) != 0U);
        cJSON_AddBoolToObject(object, "heater_on",
                              (state & TELEMETRY_STATE_HEATER_ON) != 0U);
        cJSON_AddBoolToObject(object, "motor_on",
                              (state & TELEMETRY_STATE_MOTOR_ON) != 0U);
        cJSON_AddBoolToObject(
            object, "temperature_enabled",
            (state & TELEMETRY_STATE_TEMPERATURE_ENABLED) != 0U);
        cJSON_AddBoolToObject(
            object, "humidity_enabled",
            (state & TELEMETRY_STATE_HUMIDITY_ENABLED) != 0U);
        cJSON_AddBoolToObject(object, "fan_running",
                              (state & TELEMETRY_STATE_FAN_RUNNING) != 0U);
        cJSON_AddBoolToObject(object, "fan_valid",
                              (state & TELEMETRY_STATE_FAN_VALID) != 0U);
        cJSON_AddBoolToObject(object, "remote_owned",
                              (state & TELEMETRY_STATE_REMOTE_OWNED) != 0U);
        cJSON_AddNumberToObject(object, "output_permille",
                                sample->frame.output_permille);
        cJSON_AddNumberToObject(object, "target_temperature_c",
                                sample->frame.target_temperature_c);
        cJSON_AddNumberToObject(object, "target_humidity_pct",
                                sample->frame.target_humidity_percent);
        cJSON_AddNumberToObject(object, "ambient_temperature_c",
                                sample->frame.ambient_temperature_c);
        cJSON_AddNumberToObject(object, "fault", sample->frame.fault);
        cJSON_AddStringToObject(object, "fault_name",
                                fault_name(sample->frame.fault));
        cJSON_AddNumberToObject(object, "ack_session",
                                sample->frame.ack_session);
        cJSON_AddNumberToObject(object, "ack_sequence",
                                sample->frame.ack_sequence);
        cJSON_AddNumberToObject(object, "ack_result",
                                sample->frame.ack_result);
        cJSON_AddStringToObject(
            object, "ack_result_name",
            telemetry_ack_result_name(sample->frame.ack_result));
        cJSON_AddNumberToObject(object, "command_rx_errors",
                                sample->frame.command_rx_errors);
    }
}

static void add_requested_to_object(cJSON *object,
                                    const control_command_t *requested)
{
    cJSON_AddBoolToObject(object, "running", requested->running);
    cJSON_AddBoolToObject(object, "temperature_enabled",
                          requested->temperature_enabled);
    cJSON_AddNumberToObject(object, "target_temperature_c",
                            requested->target_temperature_c);
    cJSON_AddBoolToObject(object, "humidity_enabled",
                          requested->humidity_enabled);
    cJSON_AddNumberToObject(object, "target_humidity_pct",
                            requested->target_humidity_percent);
    cJSON_AddNumberToObject(object, "ambient_temperature_c",
                            requested->ambient_temperature_c);
    cJSON_AddBoolToObject(object, "clear_fault", requested->clear_fault);
    cJSON_AddBoolToObject(object, "persist", requested->persist);
    cJSON_AddNumberToObject(object, "seen_stm32_tick_ms",
                            requested->seen_stm32_tick_ms);
}

static void add_pid_status_to_object(cJSON *object,
                                     const control_pid_status_t *status)
{
    cJSON_AddBoolToObject(object, "has_request", status->has_request);
    cJSON_AddBoolToObject(object, "pending", status->pending);
    cJSON_AddBoolToObject(object, "applied", status->applied);
    cJSON_AddBoolToObject(object, "canceled_by_stm32_reset",
                          status->canceled_by_stm32_reset);
    cJSON_AddNumberToObject(object, "result", status->result);
    cJSON_AddStringToObject(object, "result_name",
                            telemetry_ack_result_name(status->result));
    if (status->has_request) {
        cJSON_AddNumberToObject(object, "session",
                                status->requested.session);
        cJSON_AddNumberToObject(object, "sequence",
                                status->requested.sequence);
        cJSON *requested = cJSON_AddObjectToObject(object, "requested");
        cJSON_AddNumberToObject(requested, "kp",
                                status->requested.kp_tenths / 10.0);
        cJSON_AddNumberToObject(requested, "ki",
                                status->requested.ki_hundredths / 100.0);
        cJSON_AddNumberToObject(requested, "kd",
                                status->requested.kd_tenths / 10.0);
    } else {
        cJSON_AddNullToObject(object, "session");
        cJSON_AddNullToObject(object, "sequence");
        cJSON_AddNullToObject(object, "requested");
    }
}

static void add_control_to_object(cJSON *object, uint64_t now_ms)
{
    control_link_status_t status;
    control_link_get_status(now_ms, &status);
    cJSON_AddStringToObject(object, "status",
                            control_link_status_name(&status));
    cJSON_AddBoolToObject(object, "has_request", status.has_request);
    cJSON_AddBoolToObject(object, "pending", status.pending);
    cJSON_AddBoolToObject(object, "applied", status.applied);
    cJSON_AddBoolToObject(object, "canceled_by_stm32_reset",
                          status.canceled_by_stm32_reset);
    cJSON_AddNumberToObject(object, "result", status.result);
    cJSON_AddStringToObject(object, "result_name",
                            telemetry_ack_result_name(status.result));
    cJSON_AddNumberToObject(object, "command_tx_errors",
                            status.command_tx_errors);
    if (status.has_request) {
        cJSON_AddNumberToObject(object, "session",
                                status.requested.session);
        cJSON_AddNumberToObject(object, "sequence",
                                status.requested.sequence);
        cJSON *requested = cJSON_AddObjectToObject(object, "requested");
        add_requested_to_object(requested, &status.requested);
    } else {
        cJSON_AddNullToObject(object, "session");
        cJSON_AddNullToObject(object, "sequence");
        cJSON_AddNullToObject(object, "requested");
    }
    if (status.command_sent) {
        cJSON_AddNumberToObject(object, "last_send_age_ms",
                                (double)status.last_send_age_ms);
    } else {
        cJSON_AddNullToObject(object, "last_send_age_ms");
    }

    cJSON *link = cJSON_AddObjectToObject(object, "link");
    cJSON_AddBoolToObject(link, "present", status.telemetry_present);
    cJSON_AddBoolToObject(link, "extended", status.telemetry_extended);
    cJSON_AddBoolToObject(link, "fresh", status.telemetry_fresh);
    cJSON_AddNumberToObject(link, "latest_stm32_tick_ms",
                            status.latest_stm32_tick_ms);
    if (status.telemetry_present) {
        cJSON_AddNumberToObject(link, "age_ms",
                                (double)status.telemetry_age_ms);
    } else {
        cJSON_AddNullToObject(link, "age_ms");
    }
    cJSON *pid = cJSON_AddObjectToObject(object, "pid");
    add_pid_status_to_object(pid, &status.pid);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    telemetry_store_stats_t stats;
    telemetry_store_get_stats(&stats);
    wifi_manager_status_t wifi;
    wifi_manager_get_status(&wifi);
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);

    cJSON *root = cJSON_CreateObject();
    cJSON *data = cJSON_AddObjectToObject(root, "data");
    cJSON_AddBoolToObject(data, "present", stats.has_current);
    if (stats.has_current) {
        add_sample_to_object(data, &stats.current, now_ms);
    } else {
        cJSON_AddBoolToObject(data, "stale", true);
    }

    cJSON *statistics = cJSON_AddObjectToObject(root, "stats");
    cJSON_AddNumberToObject(statistics, "accepted_frames", stats.accepted_frames);
    cJSON_AddNumberToObject(statistics, "parse_errors", stats.parse_errors);
    cJSON_AddNumberToObject(statistics, "spi_errors", stats.spi_errors);
    cJSON_AddNumberToObject(statistics, "history_count", stats.history_count);
    cJSON_AddNumberToObject(statistics, "history_capacity", stats.history_capacity);
    cJSON_AddStringToObject(statistics, "last_error", stats.last_error);

    cJSON *network = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddBoolToObject(network, "sta_connected", wifi.sta_connected);
    cJSON_AddBoolToObject(network, "ap_active", wifi.ap_active);
    cJSON_AddStringToObject(network, "sta_ssid", wifi.sta_ssid);
    cJSON_AddStringToObject(network, "ap_ssid", wifi.ap_ssid);
    cJSON_AddStringToObject(network, "sta_ip", wifi.sta_ip);
    cJSON_AddStringToObject(network, "ap_ip", wifi.ap_ip);
    cJSON_AddNumberToObject(network, "rssi", wifi.rssi);
    cJSON_AddNumberToObject(root, "device_uptime_ms", (double)now_ms);
    cJSON *control = cJSON_AddObjectToObject(root, "control");
    add_control_to_object(control, now_ms);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    const esp_err_t result = send_json(request, json);
    free(json);
    return result;
}

static size_t history_limit_from_query(httpd_req_t *request)
{
    size_t limit = 300U;
    const size_t query_length = httpd_req_get_url_query_len(request);
    if (query_length == 0U || query_length >= 64U) {
        return limit;
    }
    char query[64];
    char value[16];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
        char *end = NULL;
        const unsigned long parsed = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && parsed > 0U) {
            limit = parsed > MAX_HISTORY_RESPONSE_SAMPLES
                        ? MAX_HISTORY_RESPONSE_SAMPLES
                        : (size_t)parsed;
        }
    }
    return limit;
}

static esp_err_t history_handler(httpd_req_t *request)
{
    telemetry_store_stats_t stats;
    telemetry_store_get_stats(&stats);
    const size_t limit = history_limit_from_query(request);
    const size_t response_count = stats.history_count < limit
                                      ? stats.history_count
                                      : limit;
    const size_t start = stats.history_count - response_count;

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, "[", 1U), TAG,
                        "history response start failed");
    for (size_t index = 0U; index < response_count; ++index) {
        telemetry_sample_t sample;
        if (!telemetry_store_copy_at(start + index, &sample)) {
            continue;
        }
        char json[640];
        const int length = sample_json(json, sizeof(json), &sample);
        if (length <= 0 || (size_t)length >= sizeof(json)) {
            continue;
        }
        if (index > 0U) {
            ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, ",", 1U), TAG,
                                "history separator failed");
        }
        ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, json, length), TAG,
                            "history item failed");
    }
    ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, "]", 1U), TAG,
                        "history response end failed");
    return httpd_resp_send_chunk(request, NULL, 0U);
}

static esp_err_t csv_handler(httpd_req_t *request)
{
    telemetry_store_stats_t stats;
    telemetry_store_get_stats(&stats);
    httpd_resp_set_type(request, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(request, "Content-Disposition",
                       "attachment; filename=temperature_history.csv");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    ESP_RETURN_ON_ERROR(
        httpd_resp_send_chunk(
            request,
            "sequence,received_monotonic_ms,stm32_tick_ms,temperature_c,"
            "humidity_percent,valid,extended,running,"
            "heater_on,motor_on,temperature_enabled,humidity_enabled,"
            "fan_running,fan_valid,remote_owned,output_permille,"
            "target_temperature_c,target_humidity_percent,ambient_temperature_c,"
            "fault,ack_session,"
            "ack_sequence,ack_result,command_rx_errors\n",
            HTTPD_RESP_USE_STRLEN),
        TAG, "CSV header failed");

    for (size_t index = 0U; index < stats.history_count; ++index) {
        telemetry_sample_t sample;
        if (!telemetry_store_copy_at(index, &sample)) {
            continue;
        }
        char line[320];
        int length;
        const telemetry_frame_t *frame = &sample.frame;
        char sensor_values[80];
        if (frame->valid) {
            snprintf(sensor_values, sizeof(sensor_values), "%.3f,%.3f,1",
                     frame->temperature_c,
                     frame->humidity_percent);
        } else {
            snprintf(sensor_values, sizeof(sensor_values), ",,0");
        }
        if (frame->extended) {
            const uint8_t state = frame->state_bits;
            length = snprintf(
                line, sizeof(line),
                "%" PRIu32 ",%" PRIu64 ",%" PRIu32 ",%s,1,"
                "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,"
                "%" PRIu32 ",%" PRIu32 ",%u,%" PRIu32
                "\n",
                sample.sequence, sample.received_monotonic_ms,
                frame->stm32_tick_ms, sensor_values,
                (state & TELEMETRY_STATE_RUNNING) != 0U,
                (state & TELEMETRY_STATE_HEATER_ON) != 0U,
                (state & TELEMETRY_STATE_MOTOR_ON) != 0U,
                (state & TELEMETRY_STATE_TEMPERATURE_ENABLED) != 0U,
                (state & TELEMETRY_STATE_HUMIDITY_ENABLED) != 0U,
                (state & TELEMETRY_STATE_FAN_RUNNING) != 0U,
                (state & TELEMETRY_STATE_FAN_VALID) != 0U,
                (state & TELEMETRY_STATE_REMOTE_OWNED) != 0U,
                (unsigned int)frame->output_permille,
                (unsigned int)frame->target_temperature_c,
                (unsigned int)frame->target_humidity_percent,
                (int)frame->ambient_temperature_c,
                (unsigned int)frame->fault, frame->ack_session,
                frame->ack_sequence, (unsigned int)frame->ack_result,
                frame->command_rx_errors);
        } else {
            length = snprintf(
                line, sizeof(line),
                "%" PRIu32 ",%" PRIu64 ",%" PRIu32 ",%s,0,,,,,,,,,,,,,,,,,\n",
                sample.sequence, sample.received_monotonic_ms,
                frame->stm32_tick_ms, sensor_values);
        }
        if (length > 0 && (size_t)length < sizeof(line)) {
            ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, line, length), TAG,
                                "CSV row failed");
        }
    }
    return httpd_resp_send_chunk(request, NULL, 0U);
}

static esp_err_t websocket_handler(httpd_req_t *request)
{
    if (request->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket client connected (fd=%d)",
                 httpd_req_to_sockfd(request));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t result = httpd_ws_recv_frame(request, &frame, 0U);
    if (result != ESP_OK) {
        return result;
    }
    if (frame.len > 128U) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t payload[129] = {0};
    frame.payload = payload;
    result = httpd_ws_recv_frame(request, &frame, sizeof(payload) - 1U);
    if (result != ESP_OK) {
        return result;
    }
    if (frame.type == HTTPD_WS_TYPE_PING) {
        frame.type = HTTPD_WS_TYPE_PONG;
        return httpd_ws_send_frame(request, &frame);
    }
    return ESP_OK;
}

static void websocket_broadcast_work(void *argument)
{
    websocket_message_t *message = argument;
    int client_fds[MAX_WS_CLIENTS];
    size_t client_count = MAX_WS_CLIENTS;
    if (s_server != NULL &&
        httpd_get_client_list(s_server, &client_count, client_fds) == ESP_OK) {
        httpd_ws_frame_t frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)message->json,
            .len = strlen(message->json),
        };
        for (size_t index = 0U; index < client_count; ++index) {
            if (httpd_ws_get_fd_info(s_server, client_fds[index]) ==
                HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_send_frame_async(s_server, client_fds[index], &frame);
            }
        }
    }
    free(message);
}

void web_server_publish_sample(const telemetry_sample_t *sample)
{
    if (sample == NULL || s_server == NULL) {
        return;
    }
    websocket_message_t *message = malloc(sizeof(*message));
    if (message == NULL) {
        return;
    }
    const int length = sample_json(message->json, sizeof(message->json), sample);
    if (length <= 0 || (size_t)length >= sizeof(message->json) ||
        httpd_queue_work(s_server, websocket_broadcast_work, message) != ESP_OK) {
        free(message);
    }
}

static esp_err_t send_api_error(httpd_req_t *request, const char *status,
                                const char *message)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "error", message);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    httpd_resp_set_status(request, status);
    const esp_err_t result = send_json(request, json);
    free(json);
    return result;
}

static bool request_is_json(httpd_req_t *request)
{
    const size_t length =
        httpd_req_get_hdr_value_len(request, "Content-Type");
    if (length == 0U || length >= 80U) {
        return false;
    }
    char content_type[80];
    if (httpd_req_get_hdr_value_str(request, "Content-Type", content_type,
                                    sizeof(content_type)) != ESP_OK) {
        return false;
    }
    const char *cursor = content_type;
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }
    static const char media_type[] = "application/json";
    if (strncasecmp(cursor, media_type, sizeof(media_type) - 1U) != 0) {
        return false;
    }
    cursor += sizeof(media_type) - 1U;
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }
    return *cursor == '\0' || *cursor == ';';
}

static esp_err_t receive_json_body(httpd_req_t *request, char *body,
                                   size_t body_size, size_t *received_out)
{
    if (request->content_len <= 0 ||
        (size_t)request->content_len >= body_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        const int result =
            httpd_req_recv(request, body + received,
                           (size_t)request->content_len - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (result <= 0) {
            return ESP_FAIL;
        }
        received += (size_t)result;
    }
    body[received] = '\0';
    *received_out = received;
    return ESP_OK;
}

static bool control_key_allowed(const char *name)
{
    static const char *const allowed[] = {
        "running",
        "temperature_enabled",
        "target_temperature_c",
        "humidity_enabled",
        "target_humidity_pct",
        "ambient_temperature_c",
        "clear_fault",
        "persist",
    };
    for (size_t index = 0U; index < sizeof(allowed) / sizeof(allowed[0]);
         ++index) {
        if (strcmp(name, allowed[index]) == 0) {
            return true;
        }
    }
    return false;
}

static bool pid_key_allowed(const char *name)
{
    return strcmp(name, "kp") == 0 || strcmp(name, "ki") == 0 ||
           strcmp(name, "kd") == 0;
}

static size_t json_key_count(const cJSON *root, const char *name)
{
    size_t count = 0U;
    for (const cJSON *item = root->child; item != NULL; item = item->next) {
        if (item->string != NULL && strcmp(item->string, name) == 0) {
            ++count;
        }
    }
    return count;
}

static bool get_control_bool(const cJSON *root, const char *name,
                             bool required, bool *value, char *error,
                             size_t error_size)
{
    const size_t count = json_key_count(root, name);
    if (count == 0U && !required) {
        return true;
    }
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (count != 1U || !cJSON_IsBool(item)) {
        snprintf(error, error_size, "invalid %s", name);
        return false;
    }
    *value = cJSON_IsTrue(item);
    return true;
}

static bool get_control_uint8(const cJSON *root, const char *name,
                              uint8_t minimum, uint8_t maximum,
                              uint8_t *value, char *error,
                              size_t error_size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (json_key_count(root, name) != 1U || !cJSON_IsNumber(item) ||
        !isfinite(item->valuedouble) ||
        floor(item->valuedouble) != item->valuedouble ||
        item->valuedouble < minimum || item->valuedouble > maximum) {
        snprintf(error, error_size, "invalid %s", name);
        return false;
    }
    *value = (uint8_t)item->valuedouble;
    return true;
}

static bool get_control_int8(const cJSON *root, const char *name,
                             int8_t minimum, int8_t maximum,
                             int8_t *value, char *error,
                             size_t error_size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (json_key_count(root, name) != 1U || !cJSON_IsNumber(item) ||
        !isfinite(item->valuedouble) ||
        floor(item->valuedouble) != item->valuedouble ||
        item->valuedouble < minimum || item->valuedouble > maximum) {
        snprintf(error, error_size, "invalid %s", name);
        return false;
    }
    *value = (int8_t)item->valuedouble;
    return true;
}

static bool get_pid_number(const cJSON *root, const char *name,
                           double minimum, double maximum, double scale,
                           double *value, char *error, size_t error_size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    const double scaled = item != NULL && cJSON_IsNumber(item)
                              ? item->valuedouble * scale
                              : 0.0;
    if (json_key_count(root, name) != 1U || !cJSON_IsNumber(item) ||
        !isfinite(item->valuedouble) || item->valuedouble < minimum ||
        item->valuedouble > maximum ||
        fabs(scaled - round(scaled)) > 1e-6) {
        snprintf(error, error_size, "invalid %s", name);
        return false;
    }
    *value = item->valuedouble;
    return true;
}

static bool parse_control_request(cJSON *root, control_request_t *control,
                                  char *error, size_t error_size)
{
    if (!cJSON_IsObject(root)) {
        snprintf(error, error_size, "body must be a JSON object");
        return false;
    }
    for (const cJSON *item = root->child; item != NULL; item = item->next) {
        if (item->string == NULL || !control_key_allowed(item->string)) {
            snprintf(error, error_size, "unknown control field");
            return false;
        }
    }

    memset(control, 0, sizeof(*control));
    const bool fields_valid =
        get_control_bool(root, "running", true, &control->running,
                         error, error_size) &&
        get_control_bool(root, "temperature_enabled", true,
                         &control->temperature_enabled, error,
                         error_size) &&
        get_control_uint8(root, "target_temperature_c",
                          CONTROL_MIN_TEMPERATURE_C,
                          CONTROL_MAX_TEMPERATURE_C,
                          &control->target_temperature_c, error,
                          error_size) &&
        get_control_bool(root, "humidity_enabled", true,
                         &control->humidity_enabled, error, error_size) &&
        get_control_uint8(root, "target_humidity_pct",
                          CONTROL_MIN_HUMIDITY_PERCENT,
                          CONTROL_MAX_HUMIDITY_PERCENT,
                          &control->target_humidity_percent, error,
                          error_size) &&
        get_control_int8(root, "ambient_temperature_c",
                         CONTROL_MIN_AMBIENT_TEMPERATURE_C,
                         CONTROL_MAX_AMBIENT_TEMPERATURE_C,
                         &control->ambient_temperature_c, error, error_size) &&
        get_control_bool(root, "clear_fault", false,
                         &control->clear_fault, error, error_size) &&
        get_control_bool(root, "persist", false, &control->persist,
                         error, error_size);
    if (!fields_valid) {
        return false;
    }
    if (control->running && control->persist) {
        snprintf(error, error_size,
                 "persist=true requires running=false");
        return false;
    }
    return true;
}

static bool parse_pid_request(cJSON *root, control_pid_request_t *pid,
                              char *error, size_t error_size)
{
    if (!cJSON_IsObject(root)) {
        snprintf(error, error_size, "body must be a JSON object");
        return false;
    }
    for (const cJSON *item = root->child; item != NULL; item = item->next) {
        if (item->string == NULL || !pid_key_allowed(item->string)) {
            snprintf(error, error_size, "unknown PID field");
            return false;
        }
    }
    memset(pid, 0, sizeof(*pid));
    return get_pid_number(root, "kp", 0.0,
                          CONTROL_PID_KP_MAX_TENTHS / 10.0, 10.0,
                          &pid->kp, error, error_size) &&
           get_pid_number(root, "ki", 0.0,
                          CONTROL_PID_KI_MAX_HUNDREDTHS / 100.0, 100.0,
                          &pid->ki, error, error_size) &&
           get_pid_number(root, "kd", 0.0,
                          CONTROL_PID_KD_MAX_TENTHS / 10.0, 10.0,
                          &pid->kd, error, error_size);
}

static void add_actual_control_to_object(cJSON *object,
                                         const telemetry_frame_t *frame)
{
    const uint8_t state = frame->state_bits;
    cJSON_AddBoolToObject(object, "running",
                          (state & TELEMETRY_STATE_RUNNING) != 0U);
    cJSON_AddBoolToObject(object, "heater_on",
                          (state & TELEMETRY_STATE_HEATER_ON) != 0U);
    cJSON_AddBoolToObject(object, "motor_on",
                          (state & TELEMETRY_STATE_MOTOR_ON) != 0U);
    cJSON_AddBoolToObject(
        object, "temperature_enabled",
        (state & TELEMETRY_STATE_TEMPERATURE_ENABLED) != 0U);
    cJSON_AddBoolToObject(
        object, "humidity_enabled",
        (state & TELEMETRY_STATE_HUMIDITY_ENABLED) != 0U);
    cJSON_AddBoolToObject(object, "fan_running",
                          (state & TELEMETRY_STATE_FAN_RUNNING) != 0U);
    cJSON_AddBoolToObject(object, "fan_valid",
                          (state & TELEMETRY_STATE_FAN_VALID) != 0U);
    cJSON_AddBoolToObject(object, "remote_owned",
                          (state & TELEMETRY_STATE_REMOTE_OWNED) != 0U);
    cJSON_AddNumberToObject(object, "output_permille",
                            frame->output_permille);
    cJSON_AddNumberToObject(object, "target_temperature_c",
                            frame->target_temperature_c);
    cJSON_AddNumberToObject(object, "target_humidity_pct",
                            frame->target_humidity_percent);
    cJSON_AddNumberToObject(object, "ambient_temperature_c",
                            frame->ambient_temperature_c);
    cJSON_AddNumberToObject(object, "fault", frame->fault);
    cJSON_AddStringToObject(object, "fault_name", fault_name(frame->fault));
}

static esp_err_t control_get_handler(httpd_req_t *request)
{
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    add_control_to_object(root, now_ms);

    telemetry_store_stats_t stats;
    telemetry_store_get_stats(&stats);
    if (stats.has_current && stats.current.frame.extended) {
        cJSON *actual = cJSON_AddObjectToObject(root, "actual");
        add_actual_control_to_object(actual, &stats.current.frame);
    } else {
        cJSON_AddNullToObject(root, "actual");
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    const esp_err_t result = send_json(request, json);
    free(json);
    return result;
}

static esp_err_t control_post_handler(httpd_req_t *request)
{
    if (!request_is_json(request)) {
        return send_api_error(request, "415 Unsupported Media Type",
                              "Content-Type must be application/json");
    }
    char body[513];
    size_t received = 0U;
    const esp_err_t receive_result =
        receive_json_body(request, body, sizeof(body), &received);
    if (receive_result == ESP_ERR_INVALID_SIZE) {
        return send_api_error(request, "400 Bad Request",
                              "control body must be 1-512 bytes");
    }
    if (receive_result != ESP_OK) {
        return receive_result;
    }

    const char *parse_end = NULL;
    cJSON *root = cJSON_ParseWithLengthOpts(body, received + 1U, &parse_end,
                                            true);
    if (root == NULL) {
        return send_api_error(request, "400 Bad Request",
                              "invalid JSON object");
    }
    control_request_t control;
    char error[96] = {0};
    const bool valid =
        parse_control_request(root, &control, error, sizeof(error));
    cJSON_Delete(root);
    if (!valid) {
        return send_api_error(request, "400 Bad Request", error);
    }

    uint32_t session = 0U;
    uint32_t sequence = 0U;
    const esp_err_t result =
        control_link_submit(&control, &session, &sequence);
    if (result == ESP_ERR_INVALID_STATE) {
        return send_api_error(
            request, "409 Conflict",
            "fresh extended STM32 telemetry is required to start");
    }
    if (result != ESP_OK) {
        return send_api_error(request, "400 Bad Request",
                              "invalid control request");
    }

    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddBoolToObject(response, "accepted", true);
    cJSON_AddStringToObject(response, "status", "pending");
    cJSON_AddNumberToObject(response, "session", session);
    cJSON_AddNumberToObject(response, "sequence", sequence);
    char *json = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    httpd_resp_set_status(request, "202 Accepted");
    const esp_err_t send_result = send_json(request, json);
    free(json);
    return send_result;
}

static esp_err_t pid_get_handler(httpd_req_t *request)
{
    control_link_status_t status;
    control_link_get_status((uint64_t)(esp_timer_get_time() / 1000LL),
                            &status);
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    add_pid_status_to_object(root, &status.pid);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    const esp_err_t result = send_json(request, json);
    free(json);
    return result;
}

static esp_err_t pid_post_handler(httpd_req_t *request)
{
    if (!request_is_json(request)) {
        return send_api_error(request, "415 Unsupported Media Type",
                              "Content-Type must be application/json");
    }
    char body[257];
    size_t received = 0U;
    const esp_err_t receive_result =
        receive_json_body(request, body, sizeof(body), &received);
    if (receive_result == ESP_ERR_INVALID_SIZE) {
        return send_api_error(request, "400 Bad Request",
                              "PID body must be 1-256 bytes");
    }
    if (receive_result != ESP_OK) {
        return receive_result;
    }

    const char *parse_end = NULL;
    cJSON *root = cJSON_ParseWithLengthOpts(body, received + 1U, &parse_end,
                                            true);
    if (root == NULL) {
        return send_api_error(request, "400 Bad Request", "invalid JSON object");
    }
    control_pid_request_t pid;
    char error[96] = {0};
    const bool valid = parse_pid_request(root, &pid, error, sizeof(error));
    cJSON_Delete(root);
    if (!valid) {
        return send_api_error(request, "400 Bad Request", error);
    }

    uint32_t session = 0U;
    uint32_t sequence = 0U;
    const esp_err_t result =
        control_link_submit_pid(&pid, &session, &sequence);
    if (result != ESP_OK) {
        return send_api_error(request, "400 Bad Request",
                              "invalid PID request");
    }

    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddBoolToObject(response, "accepted", true);
    cJSON_AddStringToObject(response, "status", "pending");
    cJSON_AddNumberToObject(response, "session", session);
    cJSON_AddNumberToObject(response, "sequence", sequence);
    char *json = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    httpd_resp_set_status(request, "202 Accepted");
    const esp_err_t send_result = send_json(request, json);
    free(json);
    return send_result;
}

static esp_err_t settings_get_handler(httpd_req_t *request)
{
    wifi_manager_settings_t settings;
    wifi_manager_get_settings(&settings);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "sta_ssid", settings.sta_ssid);
    cJSON_AddBoolToObject(root, "sta_password_set",
                          settings.sta_password[0] != '\0');
    cJSON_AddStringToObject(root, "ap_ssid", settings.ap_ssid);
    cJSON_AddBoolToObject(root, "ap_password_set",
                          settings.ap_password[0] != '\0');
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON allocation failed");
    }
    const esp_err_t result = send_json(request, json);
    free(json);
    return result;
}

static bool update_json_string(cJSON *root, const char *name,
                               char *destination, size_t destination_size,
                               bool required, char *error, size_t error_size)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (item == NULL) {
        if (required) {
            snprintf(error, error_size, "missing %s", name);
            return false;
        }
        return true;
    }
    if (!cJSON_IsString(item) || item->valuestring == NULL ||
        strlen(item->valuestring) >= destination_size) {
        snprintf(error, error_size, "invalid %s", name);
        return false;
    }
    snprintf(destination, destination_size, "%s", item->valuestring);
    return true;
}

static void restart_task(void *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(1000U));
    esp_restart();
}

static esp_err_t settings_post_handler(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len > 512) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Settings body must be 1-512 bytes");
    }
    char body[513];
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        const int result = httpd_req_recv(request, body + received,
                                          request->content_len - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (result <= 0) {
            return ESP_FAIL;
        }
        received += (size_t)result;
    }
    body[received] = '\0';

    cJSON *root = cJSON_ParseWithLength(body, received);
    if (root == NULL || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid JSON object");
    }

    wifi_manager_settings_t settings;
    wifi_manager_get_settings(&settings);
    char error[96] = {0};
    bool valid = update_json_string(root, "sta_ssid", settings.sta_ssid,
                                    sizeof(settings.sta_ssid), true, error,
                                    sizeof(error)) &&
                 update_json_string(root, "sta_password", settings.sta_password,
                                    sizeof(settings.sta_password), false, error,
                                    sizeof(error)) &&
                 update_json_string(root, "ap_ssid", settings.ap_ssid,
                                    sizeof(settings.ap_ssid), true, error,
                                    sizeof(error)) &&
                 update_json_string(root, "ap_password", settings.ap_password,
                                    sizeof(settings.ap_password), false, error,
                                    sizeof(error));
    cJSON_Delete(root);
    if (valid) {
        valid = wifi_manager_validate_settings(&settings, error, sizeof(error));
    }
    if (!valid) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, error);
    }
    const esp_err_t save_result = wifi_manager_save_settings(&settings);
    if (save_result != ESP_OK) {
        ESP_LOGE(TAG, "Could not save settings: %s", esp_err_to_name(save_result));
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not save settings");
    }
    const esp_err_t response = send_json(request,
                                          "{\"ok\":true,\"restarting\":true}");
    xTaskCreate(restart_task, "settings_restart", 2048, NULL, 3, NULL);
    return response;
}

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG,
                        "HTTP server start failed");

    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_handler},
        {.uri = "/settings", .method = HTTP_GET, .handler = index_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/control", .method = HTTP_GET,
         .handler = control_get_handler},
        {.uri = "/api/control", .method = HTTP_POST,
         .handler = control_post_handler},
        {.uri = "/api/pid", .method = HTTP_GET,
         .handler = pid_get_handler},
        {.uri = "/api/pid", .method = HTTP_POST,
         .handler = pid_post_handler},
        {.uri = "/api/history", .method = HTTP_GET, .handler = history_handler},
        {.uri = "/api/history.csv", .method = HTTP_GET, .handler = csv_handler},
        {.uri = "/api/settings", .method = HTTP_GET,
         .handler = settings_get_handler},
        {.uri = "/api/settings", .method = HTTP_POST,
         .handler = settings_post_handler},
        {.uri = "/ws", .method = HTTP_GET, .handler = websocket_handler,
         .is_websocket = true},
    };
    for (size_t index = 0U; index < sizeof(handlers) / sizeof(handlers[0]); ++index) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &handlers[index]),
                            TAG, "URI registration failed");
    }
    ESP_LOGI(TAG, "Dashboard listening on HTTP port %u", config.server_port);
    return ESP_OK;
}
