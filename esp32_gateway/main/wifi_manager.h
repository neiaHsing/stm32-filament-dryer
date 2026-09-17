#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define WIFI_MANAGER_SSID_MAX 32U
#define WIFI_MANAGER_PASSWORD_MAX 63U

typedef struct {
    char sta_ssid[WIFI_MANAGER_SSID_MAX + 1U];
    char sta_password[WIFI_MANAGER_PASSWORD_MAX + 1U];
    char ap_ssid[WIFI_MANAGER_SSID_MAX + 1U];
    char ap_password[WIFI_MANAGER_PASSWORD_MAX + 1U];
} wifi_manager_settings_t;

typedef struct {
    bool sta_connected;
    bool ap_active;
    int8_t rssi;
    char sta_ssid[WIFI_MANAGER_SSID_MAX + 1U];
    char ap_ssid[WIFI_MANAGER_SSID_MAX + 1U];
    char sta_ip[16];
    char ap_ip[16];
} wifi_manager_status_t;

esp_err_t wifi_manager_start(void);
void wifi_manager_get_status(wifi_manager_status_t *status);
void wifi_manager_get_settings(wifi_manager_settings_t *settings);

/* Saves NVS only. The caller should reboot after returning a response. */
esp_err_t wifi_manager_save_settings(const wifi_manager_settings_t *settings);

bool wifi_manager_validate_settings(const wifi_manager_settings_t *settings,
                                    char *error, size_t error_size);

#endif
