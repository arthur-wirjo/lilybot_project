#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "MOTOR_TEST";

// Updated Pins
#define BRA_PIN_1 42
#define BRA_PIN_2 16
#define BRA_PIN_3 15

#define FG_PIN_1 1
#define FG_PIN_2 6
#define FG_PIN_3 4

#define FR_PIN_1 39
#define FR_PIN_2 12
#define FR_PIN_3 41

#define PWM_PIN_1 38
#define PWM_PIN_2 11
#define PWM_PIN_3 40

// LEDC Configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY  30000 // Updated to 30kHz based on your hardware spec

// UART Configuration
#define COMMAND_UART_NUM UART_NUM_1
#define COMMAND_UART_TX_PIN 17
#define COMMAND_UART_RX_PIN 18
#define BUF_SIZE 256

// Calibration Constants (Tweak these based on your physical robot)
#define PULSES_PER_REV 6.0f
#define GEAR_RATIO 30.0f     // UPDATE THIS: e.g., if you have a 1:30 gearbox
#define WHEEL_RADIUS 0.05f   // UPDATE THIS: Wheel radius in meters

const int BRA_PINS[3] = {BRA_PIN_1, BRA_PIN_2, BRA_PIN_3};
const int FR_PINS[3]  = {FR_PIN_1, FR_PIN_2, FR_PIN_3};
const int PWM_PINS[3] = {PWM_PIN_1, PWM_PIN_2, PWM_PIN_3};
const int FG_PINS[3]  = {FG_PIN_1, FG_PIN_2, FG_PIN_3};
const ledc_channel_t PWM_CHANNELS[3] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

static pcnt_unit_handle_t pcnt_units[3];
static int motor_directions[3] = {1, 1, 1}; // Used to sign the encoder counts

void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(COMMAND_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(COMMAND_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(COMMAND_UART_NUM, COMMAND_UART_TX_PIN, COMMAND_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized. Send commands like 'm1 100' or 'm4 0'");
}

void init_motors(void) {
    ESP_LOGI(TAG, "Initializing Motor GPIOs, PWM, and Encoders");

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BRA_PIN_1) | (1ULL << BRA_PIN_2) | (1ULL << BRA_PIN_3) |
                        (1ULL << FR_PIN_1) | (1ULL << FR_PIN_2) | (1ULL << FR_PIN_3),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    for (int i = 0; i < 3; i++) {
        gpio_set_level(BRA_PINS[i], 0);
        gpio_set_level(FR_PINS[i], 0);
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK   
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

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
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    // Initialize Pulse Counters for FG pins
    for (int i = 0; i < 3; i++) {
        pcnt_unit_config_t unit_config = {
            .high_limit = 32767,
            .low_limit = -32768,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_units[i]));

        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_units[i], &filter_config));

        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = FG_PINS[i],
            .level_gpio_num = -1, 
        };
        pcnt_channel_handle_t pcnt_chan = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_config, &pcnt_chan));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[i]));
    }
}

void set_motor_speed(int id, int speed) {
    if (id < 1 || id > 4) return;

    if (id == 4) {
        set_motor_speed(1, speed);
        set_motor_speed(2, speed);
        set_motor_speed(3, speed);
        return;
    }

    if (speed > 100) speed = 100;
    else if (speed < -100) speed = -100;

    int index = id - 1;

    if (speed == 0) {
        ledc_set_duty(LEDC_MODE, PWM_CHANNELS[index], 0);
        ledc_update_duty(LEDC_MODE, PWM_CHANNELS[index]);
        gpio_set_level(BRA_PINS[index], 0);
        motor_directions[index] = 1;
    } else {
        gpio_set_level(BRA_PINS[index], 1);
        
        if (speed > 0) {
            gpio_set_level(FR_PINS[index], 0);
            motor_directions[index] = 1;
        } else {
            gpio_set_level(FR_PINS[index], 1);
            motor_directions[index] = -1;
        }

        uint32_t duty = (abs(speed) * 1023) / 100;
        ledc_set_duty(LEDC_MODE, PWM_CHANNELS[index], duty);
        ledc_update_duty(LEDC_MODE, PWM_CHANNELS[index]);
    }
}

void uart_command_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    char line[BUF_SIZE];
    int pos = 0;
    int id, speed;

    while (1) {
        int len = uart_read_bytes(COMMAND_UART_NUM, data, (BUF_SIZE - 1), pdMS_TO_TICKS(20));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];

                if (c == '\n' || c == '\r') {
                    if (pos > 0) {
                        line[pos] = '\0';

                        if (sscanf(line, "m%d %d", &id, &speed) == 2) {
                            ESP_LOGI(TAG, "Executing: Motor %d -> %d%%", id, speed);
                            set_motor_speed(id, speed);
                        } else {
                            ESP_LOGW(TAG, "Format: m<id> <speed> (e.g., m1 50)");
                        }
                        pos = 0;
                    }
                } else if (pos < BUF_SIZE - 1) {
                    line[pos++] = c;
                }
            }
        }
    }
}

// Task to read encoders and print calibration data every 1 second
void encoder_monitor_task(void *pvParameters) {
    const float dt = 1.0f; // 1 second interval
    int pulse_counts[3] = {0};

    while (1) {
        printf("\n--- Encoder Calibration Stats ---\n");
        for (int i = 0; i < 3; i++) {
            pcnt_unit_get_count(pcnt_units[i], &pulse_counts[i]);
            pcnt_unit_clear_count(pcnt_units[i]); 

            int signed_pulses = pulse_counts[i] * motor_directions[i];
            
            // Calculate Revolutions Per Minute (RPM) of the output shaft
            float revs = (float)signed_pulses / (PULSES_PER_REV * GEAR_RATIO);
            float rpm = (revs / dt) * 60.0f;

            // Calculate Linear Speed in m/s
            float speed_ms = (revs * 2.0f * M_PI * WHEEL_RADIUS) / dt;

            printf("Motor %d | Pulses/sec: %4d | RPM: %6.1f | Speed: %5.2f m/s\n", 
                   i + 1, signed_pulses, rpm, speed_ms);
        }
        printf("---------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    init_motors();
    init_uart();
    xTaskCreate(uart_command_task, "uart_command_task", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_monitor_task, "encoder_monitor_task", 4096, NULL, 4, NULL);
}