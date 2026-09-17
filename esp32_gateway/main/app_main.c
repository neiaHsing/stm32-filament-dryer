#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "control_link.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "nvs_flash.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include "telemetry_protocol.h"
#include "telemetry_store.h"
#include "web_server.h"
#include "wifi_manager.h"

#define STM32_SPI_SCLK_GPIO GPIO_NUM_18
#define STM32_SPI_SDIO_GPIO GPIO_NUM_23
#define STM32_SPI_CS_GPIO GPIO_NUM_27

#define SPI_RX_TIMEOUT_MS 2500U
#define SPI_TX_TIMEOUT_MS 300U
#define SPI_IDLE_RESYNC_MS 100U

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define GATEWAY_SPI_HOST SPI3_HOST
#else
#define GATEWAY_SPI_HOST VSPI_HOST
#endif

static const char *TAG = "temperature_gateway";
static WORD_ALIGNED_ATTR uint8_t
    s_spi_rx_buffer[TELEMETRY_FRAME_MAX_BYTES];
static WORD_ALIGNED_ATTR uint8_t
    s_spi_tx_buffer[CONTROL_COMMAND_SLOT_BYTES];
_Static_assert(TELEMETRY_FRAME_MAX_BYTES == CONTROL_COMMAND_SLOT_BYTES,
               "Request and reply slots must both fit the 64-byte FIFO");
/*
 * spi_slave_transmit() can time out while its descriptor is still mounted in
 * the driver. Keep both descriptors alive for the lifetime of the firmware;
 * the recovery path cancels the mounted transaction before reusing it.
 */
static spi_slave_transaction_t s_spi_rx_transaction;
static spi_slave_transaction_t s_spi_tx_transaction;

/*
 * GPIO23 is connected to both the slave SPID input and SPIQ output through
 * the GPIO matrix. Force its output-enable control to the GPIO register so
 * these callbacks decide exactly when the shared wire is driven.
 */
static void IRAM_ATTR set_sdio_output_enabled(bool enabled)
{
    /* Disconnect SPIQ from the pad while receiving.  Merely clearing the GPIO
     * output-enable bit is not sufficient on classic ESP32: the peripheral's
     * CS-driven output path can briefly reclaim the pad after the transaction
     * has been armed, which clamps the shared SDIO line low. */
    gpio_ll_set_output_signal_matrix_source(
        &GPIO, STM32_SPI_SDIO_GPIO,
        enabled ? VSPIQ_OUT_IDX : SIG_GPIO_OUT_IDX, false);
    gpio_ll_set_output_enable_ctrl(&GPIO, STM32_SPI_SDIO_GPIO, false, false);
    if (enabled) {
        gpio_ll_output_enable(&GPIO, STM32_SPI_SDIO_GPIO);
    } else {
        gpio_ll_output_disable(&GPIO, STM32_SPI_SDIO_GPIO);
    }
}

static void IRAM_ATTR spi_post_setup(spi_slave_transaction_t *transaction)
{
    set_sdio_output_enabled(transaction != NULL &&
                            transaction->tx_buffer != NULL);
}

static void IRAM_ATTR spi_post_transaction(
    spi_slave_transaction_t *transaction)
{
    (void)transaction;
    set_sdio_output_enabled(false);
}

static esp_err_t initialize_spi_slave(void)
{
    const spi_bus_config_t bus = {
        .mosi_io_num = STM32_SPI_SDIO_GPIO,
        .miso_io_num = STM32_SPI_SDIO_GPIO,
        .sclk_io_num = STM32_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TELEMETRY_FRAME_MAX_BYTES,
        .flags = SPICOMMON_BUSFLAG_SLAVE | SPICOMMON_BUSFLAG_GPIO_PINS,
        .intr_flags = 0,
    };
    const spi_slave_interface_config_t slave = {
        .spics_io_num = STM32_SPI_CS_GPIO,
        .flags = 0,
        .queue_size = 1,
        /* Mode 1 matches the STM32 master and is supported without DMA. */
        .mode = 1,
        .post_setup_cb = spi_post_setup,
        .post_trans_cb = spi_post_transaction,
    };

    /* Both directions are exactly 64 bytes, which fits the classic ESP32 SPI
     * hardware FIFO. Keeping DMA disabled also avoids the observed failure
     * mode where only the first 64 bytes of a larger request reached RAM. */
    esp_err_t result = spi_slave_initialize(GATEWAY_SPI_HOST, &bus, &slave,
                                            SPI_DMA_DISABLED);
    if (result != ESP_OK) {
        return result;
    }
    set_sdio_output_enabled(false);

    result = gpio_set_pull_mode(STM32_SPI_CS_GPIO, GPIO_PULLUP_ONLY);
    if (result == ESP_OK) {
        result = gpio_set_pull_mode(STM32_SPI_SDIO_GPIO, GPIO_PULLUP_ONLY);
    }
    if (result != ESP_OK) {
        set_sdio_output_enabled(false);
        spi_slave_free(GATEWAY_SPI_HOST);
    }
    return result;
}

static uint64_t monotonic_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000LL);
}

/* A long CS-high window exists only between complete request/reply pairs. */
static void wait_for_idle_slot_boundary(void)
{
    uint64_t high_since_ms = 0U;
    const TickType_t one_ms = pdMS_TO_TICKS(1U);
    const TickType_t poll_delay = one_ms == 0U ? 1U : one_ms;
    for (;;) {
        const uint64_t now_ms = monotonic_ms();
        if (gpio_get_level(STM32_SPI_CS_GPIO) == 0) {
            high_since_ms = 0U;
        } else if (high_since_ms == 0U) {
            high_since_ms = now_ms;
        } else if (now_ms - high_since_ms >= SPI_IDLE_RESYNC_MS) {
            return;
        }
        vTaskDelay(poll_delay);
    }
}

static void resynchronize_spi_slave(void)
{
    set_sdio_output_enabled(false);
    wait_for_idle_slot_boundary();

    /* No CS edge has occurred for 100 ms, so no master transaction can be
     * active. The private reset API disables the ISR, drops any descriptor
     * left mounted after a timeout and deliberately raises trans_done. The
     * next queue operation re-enables the ISR and loads a fresh RX slot. */
    const esp_err_t reset_result = spi_slave_queue_reset(GATEWAY_SPI_HOST);
    if (reset_result != ESP_OK) {
        ESP_LOGE(TAG, "Could not cancel SPI transaction: %s; restarting",
                 esp_err_to_name(reset_result));
        esp_restart();
    }

    /* A transaction may have completed after spi_slave_transmit() timed out
     * but before the stable-high window elapsed. Remove that stale return
     * entry so the next blocking transmit cannot consume the old descriptor. */
    for (;;) {
        spi_slave_transaction_t *discarded = NULL;
        const esp_err_t drain_result = spi_slave_get_trans_result(
            GATEWAY_SPI_HOST, &discarded, 0U);
        if (drain_result == ESP_ERR_TIMEOUT) {
            break;
        }
        if (drain_result != ESP_OK &&
            drain_result != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Could not drain SPI result: %s; restarting",
                     esp_err_to_name(drain_result));
            esp_restart();
        }
    }
    ESP_LOGI(TAG, "Half-duplex SPI link resynchronized");
}

static bool transmit_reply_slot(void)
{
    s_spi_tx_transaction = (spi_slave_transaction_t){
        .length = sizeof(s_spi_tx_buffer) * 8U,
        .tx_buffer = s_spi_tx_buffer,
    };
    const esp_err_t result = spi_slave_transmit(
        GATEWAY_SPI_HOST, &s_spi_tx_transaction,
        pdMS_TO_TICKS(SPI_TX_TIMEOUT_MS));
    set_sdio_output_enabled(false);
    if (result != ESP_OK) {
        telemetry_store_note_spi_error(result);
        control_link_note_command_tx_error();
        ESP_LOGW(TAG, "SPI reply slot failed: %s", esp_err_to_name(result));
        return false;
    }
    if (s_spi_tx_transaction.trans_len != sizeof(s_spi_tx_buffer) * 8U) {
        telemetry_store_note_spi_error(ESP_ERR_INVALID_SIZE);
        control_link_note_command_tx_error();
        ESP_LOGW(TAG, "SPI reply slot was %u bits, expected %u",
                 (unsigned int)s_spi_tx_transaction.trans_len,
                 (unsigned int)(sizeof(s_spi_tx_buffer) * 8U));
        return false;
    }
    return true;
}

static void publish_telemetry(const telemetry_frame_t *frame,
                              uint64_t received_ms)
{
    telemetry_store_add(frame, received_ms);
    telemetry_store_stats_t stats;
    telemetry_store_get_stats(&stats);
    if (stats.has_current) {
        web_server_publish_sample(&stats.current);
    }
}

static bool buffer_is_all_ff(const uint8_t *buffer, size_t length)
{
    for (size_t index = 0U; index < length; ++index) {
        if (buffer[index] != UINT8_MAX) {
            return false;
        }
    }
    return true;
}

static void spi_exchange_task(void *argument)
{
    (void)argument;
    set_sdio_output_enabled(false);

    for (;;) {
        memset(s_spi_rx_buffer, 0, sizeof(s_spi_rx_buffer));
        s_spi_rx_transaction = (spi_slave_transaction_t){
            .length = sizeof(s_spi_rx_buffer) * 8U,
            .rx_buffer = s_spi_rx_buffer,
        };
        const esp_err_t receive_result = spi_slave_transmit(
            GATEWAY_SPI_HOST, &s_spi_rx_transaction,
            pdMS_TO_TICKS(SPI_RX_TIMEOUT_MS));
        if (receive_result != ESP_OK) {
            telemetry_store_note_spi_error(receive_result);
            ESP_LOGW(TAG, "SPI request slot failed: %s",
                     esp_err_to_name(receive_result));
            resynchronize_spi_slave();
            continue;
        }

        telemetry_frame_t frame = {0};
        telemetry_parse_result_t parse_result = TELEMETRY_PARSE_INCOMPLETE;
        const bool complete_slot =
            s_spi_rx_transaction.trans_len == sizeof(s_spi_rx_buffer) * 8U;
        const bool synchronization_probe =
            complete_slot &&
            buffer_is_all_ff(s_spi_rx_buffer, sizeof(s_spi_rx_buffer));
        if (complete_slot && !synchronization_probe) {
            parse_result = telemetry_parse_frame(
                s_spi_rx_buffer, sizeof(s_spi_rx_buffer), &frame);
        }

        const uint64_t received_ms = monotonic_ms();
        const bool telemetry_valid = parse_result == TELEMETRY_PARSE_OK;
        if (telemetry_valid) {
            control_link_on_telemetry(&frame, received_ms);
            (void)control_link_prepare_command_slot(
                s_spi_tx_buffer, sizeof(s_spi_tx_buffer), received_ms);
        } else {
            (void)control_reply_encode_nop(s_spi_tx_buffer,
                                           sizeof(s_spi_tx_buffer));
            if (synchronization_probe) {
                ESP_LOGD(TAG, "Answered STM32 synchronization probe");
            } else {
                telemetry_store_note_parse_error(parse_result);
                ESP_LOGW(
                    TAG, "RX slot was %u bits and parsed as %s; replying NOP",
                    (unsigned int)s_spi_rx_transaction.trans_len,
                    telemetry_parse_result_name(parse_result));
            }
        }

        /*
         * Every completed RX transaction gets exactly one verifiable reply.
         * This lets an independently reset STM32 probe once or twice until it
         * sees a CRC-protected NOP and knows the ESP32 has returned to RX.
         */
        if (!transmit_reply_slot()) {
            if (telemetry_valid) {
                publish_telemetry(&frame, received_ms);
            }
            resynchronize_spi_slave();
            continue;
        }
        if (telemetry_valid) {
            publish_telemetry(&frame, received_ms);
        }
    }
}

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    ESP_ERROR_CHECK(telemetry_store_init(CONFIG_GATEWAY_HISTORY_SAMPLES));
    ESP_ERROR_CHECK(control_link_init());
    ESP_ERROR_CHECK(wifi_manager_start());
    ESP_ERROR_CHECK(web_server_start());
    ESP_ERROR_CHECK(initialize_spi_slave());

    BaseType_t task_result = xTaskCreate(spi_exchange_task, "spi_exchange",
                                         4096, NULL, 6, NULL);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Could not create SPI exchange task");
        abort();
    }

    ESP_LOGI(TAG,
             "Ready: PB13->GPIO18 SCLK, PB14->GPIO27 CS, "
             "PB15<->GPIO23 half-duplex SDIO");
}
