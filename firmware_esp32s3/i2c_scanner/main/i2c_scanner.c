#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "driver/gpio.h"

// --- PIN CONFIGURATION ---
#define I2C_MASTER_SDA_IO  8
#define I2C_MASTER_SCL_IO  9
#define I2C_MASTER_NUM     I2C_NUM_0

// Brake Pins (Keep these off/low)
#define BRAKE_PIN_1  42
#define BRAKE_PIN_2  15
#define BRAKE_PIN_3  16

i2c_master_bus_handle_t bus_handle;

static void i2c_bus_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    vTaskDelay(pdMS_TO_TICKS(200)); // Allow voltage to stabilize
}

void app_main(void)
{
    // 1. Set Brakes to 0 (OFF) immediately
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BRAKE_PIN_1) | (1ULL << BRAKE_PIN_2) | (1ULL << BRAKE_PIN_3);
    gpio_config(&io_conf);
    gpio_set_level(BRAKE_PIN_1, 0);
    gpio_set_level(BRAKE_PIN_2, 0);
    gpio_set_level(BRAKE_PIN_3, 0);

    // 2. Initialize I2C Bus
    i2c_bus_init();

    printf("\n\n--- STARTING I2C SCANNER ---\n");
    printf("Scanning SDA: GPIO %d, SCL: GPIO %d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    int devices_found = 0;

    // 3. Scan all valid addresses (0x08 to 0x77)
    for (uint8_t address = 0x08; address < 0x78; address++) {
        esp_err_t ret = i2c_master_probe(bus_handle, address, 50);
        
        if (ret == ESP_OK) {
            printf("-> DEVICE FOUND AT ADDRESS: 0x%02X\n", address);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        printf("\nNO DEVICES FOUND.\n");
        printf("Troubleshooting:\n");
        printf("1. Swap SDA (8) and SCL (9) wires.\n");
        printf("2. Check if MPU9250 has power (LED on?).\n");
        printf("3. Check if AD0 pin is connected to 3.3V (Address becomes 0x69).\n");
    } else {
        printf("\nScan Complete. Found %d device(s).\n", devices_found);
    }

    // Prevent reboot
    while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
}