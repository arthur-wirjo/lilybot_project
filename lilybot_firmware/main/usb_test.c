#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"

static const char *TAG = "usb_cdc";

static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    (void) event;

    uint8_t buf[64];
    size_t rx = 0;

    esp_err_t err = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, buf, sizeof(buf), &rx);
    if (err != ESP_OK || rx == 0) {
        return;
    }

    // Echo back
    tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, buf, rx);
    tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 0);
}

static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    if (event->type != CDC_EVENT_LINE_STATE_CHANGED) return;
    ESP_LOGI(TAG, "CDC%d line state: DTR=%d RTS=%d",
             itf,
             event->line_state_changed_data.dtr,
             event->line_state_changed_data.rts);
}

void app_main(void)
{
    // Install TinyUSB driver (descriptors come from Kconfig defaults)
    const tinyusb_config_t tusb_cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = {
            .skip_setup = false,
            .self_powered = false,
            .vbus_monitor_io = -1,
        },
        .task = {
            .size = 4096,
            .priority = 5,
            .xCoreID = 0,
        },
        // .descriptor left as zero-init => use Kconfig-provided defaults
        .event_cb = NULL,
        .event_arg = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Init CDC ACM 0
    const tinyusb_config_cdcacm_t cdc_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc_cfg));

    ESP_LOGI(TAG, "TinyUSB CDC initialized");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}