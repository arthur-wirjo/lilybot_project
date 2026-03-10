// Documentation to self: motor initialization written
// set_motor_speed and task not yet written, please finish it

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc"

// Brake Pins
#define BRA_PIN_1 42
#define BRA_PIN_2 15
#define BRA_PIN_3 16

// Frequency Generator Pins
#define FG_PIN_1 1
#define FG_PIN_2 4
#define FG_PIN_3 6

// Forward/Reverse Pins
#define FR_PIN_1 39
#define FR_PIN_2 41
#define FR_PIN_3 12

// PWM Pins
#define PWM_PIN_1 38
#define PWM_PIN_2 40
#define PWM_PIN_3 11

// LEDC Configuration
#define LEDC_TIMER      LED_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY  20000

const int BRA_PINS[3] = {BRA_PIN_1, BRA_PIN_2, BRA_PIN_3};
const int FR_PINS[3] = {FR_PIN_1, FR_PIN_2, FR_PIN_3};
const int PWM_PINS[3] = {PWM_PIN_1, PWM_PIN_2, PWM_PIN_3};
const ledc_channel_t PWM_CHANNELS[3] = {LED_CHANNEL_0, LED_CHANNEL_1, LED_CHANNEL_2};

void init_motors(void) {
    ESP_LOGI("Initializing Motor GPIOs and PWM\n");

    // Configure BRA and FR pins as outputs
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BRA_PIN_1) | (1ULL << BRA_PIN_2) | (1ULL << BRA_PIN_3) |
                        (1ULL << FR_PIN_1) | (1ULL << FR_PIN_2) | (1ULL << FR_PIN_3),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Initialize BRA and FR pins
    for (int i = 0; i < 3; i++) {
        gpio_set_level(BRA_PINS[i], 0);
        gpio_set_level(FR_PINS[i], 0);
    }

    // Configure LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK   
    }
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC Channels 
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_MODE,
            .channel = PWM_CHANNELS[i],
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = PWM_PINS[i],
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(led_channel_config(&ledc_channel));
    }

    ESP_LOGI("Motor initialization complete");
}

void set_motor_speed(int id, int speed) {

}

void app_main(void)
{

}
