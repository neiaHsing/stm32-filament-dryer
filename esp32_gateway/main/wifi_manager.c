#include "wifi_manager.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#define SETTINGS_NAMESPACE "gateway"

static const char *TAG = "wifi_manager";

static SemaphoreHandle_t s_mutex;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static wifi_manager_settings_t s_settings;
static bool s_sta_connected;
static bool s_ap_active;
static uint32_t s_disconnect_count;

static void copy_text(char *destination, size_t destination_size,
                      const char *source)
{
    if (destination_size == 0U) {
        return;
    }
    snprintf(destination, destination_size, "%s", source != NULL ? source : "");
}

static void load_nvs_string(nvs_handle_t handle, const char *key,
                            char *destination, size_t destination_size)
{
    size_t required = destination_size;
    const esp_err_t result = nvs_get_str(handle, key, destination, &required);
    if (result != ESP_OK) {
        ESP_LOGD(TAG, "NVS key %s uses compiled default", key);
    }
}

static void load_settings(void)
{
    memset(&s_settings, 0, sizeof(s_settings));
    copy_text(s_settings.sta_ssid, sizeof(s_settings.sta_ssid),
              CONFIG_GATEWAY_WIFI_SSID);
    copy_text(s_settings.sta_password, sizeof(s_settings.sta_password),
              CONFIG_GATEWAY_WIFI_PASSWORD);
    copy_text(s_settings.ap_ssid, sizeof(s_settings.ap_ssid),
              CONFIG_GATEWAY_AP_SSID);
    copy_text(s_settings.ap_password, sizeof(s_settings.ap_password),
              CONFIG_GATEWAY_AP_PASSWORD);

    nvs_handle_t handle;
    uint8_t stored_revision = 0U;
    bool have_stored_revision = false;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        load_nvs_string(handle, "sta_ssid", s_settings.sta_ssid,
                        sizeof(s_settings.sta_ssid));
        load_nvs_string(handle, "sta_pass", s_settings.sta_password,
                        sizeof(s_settings.sta_password));
        load_nvs_string(handle, "ap_ssid", s_settings.ap_ssid,
                        sizeof(s_settings.ap_ssid));
        load_nvs_string(handle, "ap_pass", s_settings.ap_password,
                        sizeof(s_settings.ap_password));
        have_stored_revision = nvs_get_u8(handle, "wifi_def_rev",
                                          &stored_revision) == ESP_OK;
        nvs_close(handle);
    }

    /* A compiled credential change must take effect even when an older
     * firmware left a stale station password in NVS. After this one-time
     * migration, Settings page edits continue to be authoritative. */
    if (!have_stored_revision ||
        stored_revision != CONFIG_GATEWAY_WIFI_DEFAULTS_REVISION) {
        copy_text(s_settings.sta_ssid, sizeof(s_settings.sta_ssid),
                  CONFIG_GATEWAY_WIFI_SSID);
        copy_text(s_settings.sta_password, sizeof(s_settings.sta_password),
                  CONFIG_GATEWAY_WIFI_PASSWORD);
        if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            esp_err_t migration_result = nvs_set_str(
                handle, "sta_ssid", s_settings.sta_ssid);
            if (migration_result == ESP_OK) {
                migration_result = nvs_set_str(handle, "sta_pass",
                                               s_settings.sta_password);
            }
            if (migration_result == ESP_OK) {
                migration_result = nvs_set_u8(
                    handle, "wifi_def_rev",
                    CONFIG_GATEWAY_WIFI_DEFAULTS_REVISION);
            }
            if (migration_result == ESP_OK) {
                migration_result = nvs_commit(handle);
            }
            nvs_close(handle);
            if (migration_result != ESP_OK) {
                ESP_LOGW(TAG, "Could not persist compiled Wi-Fi defaults: %s",
                         esp_err_to_name(migration_result));
            } else {
                ESP_LOGI(TAG, "Applied compiled Wi-Fi defaults revision %u",
                         CONFIG_GATEWAY_WIFI_DEFAULTS_REVISION);
            }
        }
    }

    char error[64];
    if (!wifi_manager_validate_settings(&s_settings, error, sizeof(error))) {
        ESP_LOGW(TAG, "Stored Wi-Fi settings invalid (%s); restoring AP defaults",
                 error);
        copy_text(s_settings.ap_ssid, sizeof(s_settings.ap_ssid),
                  "Temperature-Gateway");
        copy_text(s_settings.ap_password, sizeof(s_settings.ap_password),
                  "temperature");
    }
}

bool wifi_manager_validate_settings(const wifi_manager_settings_t *settings,
                                    char *error, size_t error_size)
{
    if (settings == NULL) {
        copy_text(error, error_size, "missing settings");
        return false;
    }
    const size_t sta_ssid_length = strnlen(settings->sta_ssid,
                                           sizeof(settings->sta_ssid));
    const size_t sta_password_length = strnlen(settings->sta_password,
                                               sizeof(settings->sta_password));
    const size_t ap_ssid_length = strnlen(settings->ap_ssid,
                                          sizeof(settings->ap_ssid));
    const size_t ap_password_length = strnlen(settings->ap_password,
                                              sizeof(settings->ap_password));
    if (sta_ssid_length > WIFI_MANAGER_SSID_MAX ||
        sta_password_length > WIFI_MANAGER_PASSWORD_MAX) {
        copy_text(error, error_size, "station credentials too long");
        return false;
    }
    if (ap_ssid_length == 0U || ap_ssid_length > WIFI_MANAGER_SSID_MAX) {
        copy_text(error, error_size, "AP SSID must contain 1-32 characters");
        return false;
    }
    if (ap_password_length < 8U ||
        ap_password_length > WIFI_MANAGER_PASSWORD_MAX) {
        copy_text(error, error_size, "AP password must contain 8-63 characters");
        return false;
    }
    copy_text(error, error_size, "");
    return true;
}

static esp_err_t configure_ap(void)
{
    wifi_config_t config = {0};
    config.ap.ssid_len = strlen(s_settings.ap_ssid);
    memcpy(config.ap.ssid, s_settings.ap_ssid, config.ap.ssid_len);
    copy_text((char *)config.ap.password, sizeof(config.ap.password),
              s_settings.ap_password);
    config.ap.channel = 1;
    config.ap.max_connection = 4;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_AP, &config);
}

static void event_handler(void *argument, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    (void)argument;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        const esp_err_t result = esp_wifi_connect();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Initial STA connect failed: %s", esp_err_to_name(result));
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *disconnected = event_data;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_sta_connected = false;
        ++s_disconnect_count;
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "STA disconnected (reason=%d, attempt=%" PRIu32 ")",
                 disconnected != NULL ? disconnected->reason : -1,
                 s_disconnect_count);
        if (s_settings.sta_ssid[0] != '\0') {
            const esp_err_t result = esp_wifi_connect();
            if (result != ESP_OK) {
                ESP_LOGD(TAG, "STA reconnect deferred: %s", esp_err_to_name(result));
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got_ip = event_data;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_sta_connected = true;
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "Connected to '%s', IP: " IPSTR,
                 s_settings.sta_ssid, IP2STR(&got_ip->ip_info.ip));
    }
}

esp_err_t wifi_manager_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    load_settings();

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    esp_err_t loop_result = esp_event_loop_create_default();
    if (loop_result != ESP_OK && loop_result != ESP_ERR_INVALID_STATE) {
        return loop_result;
    }
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_sta_netif == NULL || s_ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "esp_wifi_init failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL),
        TAG, "Wi-Fi event handler registration failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL),
        TAG, "IP event handler registration failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG,
                        "Wi-Fi storage setup failed");
    /* This is a mains-powered gateway. Avoid modem-sleep gaps during the
     * STM32's one-second telemetry exchange and make reconnect timing stable. */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG,
                        "Wi-Fi power-save setup failed");

    if (s_settings.sta_ssid[0] == '\0') {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_ap_active = true;
        xSemaphoreGive(s_mutex);
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG,
                            "AP mode setup failed");
        ESP_RETURN_ON_ERROR(configure_ap(), TAG, "AP configuration failed");
        ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
        ESP_LOGI(TAG, "Demo AP '%s' active at 192.168.4.1 (no STA configured)",
                 s_settings.ap_ssid);
        return ESP_OK;
    }

    wifi_config_t station_config = {0};
    memcpy(station_config.sta.ssid, s_settings.sta_ssid,
           strlen(s_settings.sta_ssid));
    copy_text((char *)station_config.sta.password,
              sizeof(station_config.sta.password), s_settings.sta_password);
    station_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    station_config.sta.pmf_cfg.capable = true;
    station_config.sta.pmf_cfg.required = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ap_active = true;
    xSemaphoreGive(s_mutex);
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG,
                        "AP+STA mode setup failed");
    ESP_RETURN_ON_ERROR(configure_ap(), TAG, "AP configuration failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &station_config), TAG,
                        "STA configuration failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    ESP_LOGI(TAG,
             "Demo AP '%s' active at 192.168.4.1; STA join started for '%s' "
             "(password configured=%s)",
             s_settings.ap_ssid, s_settings.sta_ssid,
             s_settings.sta_password[0] != '\0' ? "yes" : "no");
    return ESP_OK;
}

void wifi_manager_get_settings(wifi_manager_settings_t *settings)
{
    if (settings == NULL || s_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *settings = s_settings;
    xSemaphoreGive(s_mutex);
}

void wifi_manager_get_status(wifi_manager_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->rssi = 0;
    if (s_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    status->sta_connected = s_sta_connected;
    status->ap_active = s_ap_active;
    copy_text(status->sta_ssid, sizeof(status->sta_ssid), s_settings.sta_ssid);
    copy_text(status->ap_ssid, sizeof(status->ap_ssid), s_settings.ap_ssid);
    xSemaphoreGive(s_mutex);

    esp_netif_ip_info_t ip_info;
    if (status->sta_connected && s_sta_netif != NULL &&
        esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, status->sta_ip, sizeof(status->sta_ip));
        wifi_ap_record_t access_point;
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            status->rssi = access_point.rssi;
        }
    }
    if (status->ap_active && s_ap_netif != NULL &&
        esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, status->ap_ip, sizeof(status->ap_ip));
    }
}

esp_err_t wifi_manager_save_settings(const wifi_manager_settings_t *settings)
{
    char error[64];
    if (!wifi_manager_validate_settings(settings, error, sizeof(error))) {
        ESP_LOGW(TAG, "Refusing invalid Wi-Fi settings: %s", error);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }
    if ((result = nvs_set_str(handle, "sta_ssid", settings->sta_ssid)) == ESP_OK &&
        (result = nvs_set_str(handle, "sta_pass", settings->sta_password)) == ESP_OK &&
        (result = nvs_set_str(handle, "ap_ssid", settings->ap_ssid)) == ESP_OK &&
        (result = nvs_set_str(handle, "ap_pass", settings->ap_password)) == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}
