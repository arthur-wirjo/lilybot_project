#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"

static const char *TAG = "usb_cdc";

static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    (void)event;

    uint8_t buf[64];
    size_t rx_size = 0;

    esp_err_t err = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
    if (err != ESP_OK || rx_size == 0) return;

    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    tinyusb_cdcacm_write_flush(itf, 0);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "initializing TinyUSB.....");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t cdc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc_cfg));

    ESP_LOGI(TAG, "USB CDC ready. Connect from the Pi to the new /dev/ttyACM*")
    
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
