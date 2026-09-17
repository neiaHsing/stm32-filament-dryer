#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "telemetry_store.h"

#include "esp_err.h"

esp_err_t web_server_start(void);

/* Queues a WebSocket update without blocking the SPI receive task. */
void web_server_publish_sample(const telemetry_sample_t *sample);

#endif
