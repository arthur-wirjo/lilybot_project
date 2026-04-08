#include "motor_control.h"
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "MOTOR_CTRL";

// Brake Pins
#define BRA_PIN_1 42
#define BRA_PIN_2 16
#define BRA_PIN_3 15

// Frequency Generator Pins
#define FG_PIN_1 1
#define FG_PIN_2 6
#define FG_PIN_3 4

// Forward/Reverse Pins
#define FR_PIN_1 39
#define FR_PIN_2 12
#define FR_PIN_3 41

// PWM Pins
#define PWM_PIN_1 38
#define PWM_PIN_2 11
#define PWM_PIN_3 40

// LEDC Configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY  20000 // Need to determine if 30KHz is better or worse

const int BRA_PINS[3] = {BRA_PIN_1, BRA_PIN_2, BRA_PIN_3};
const int FG_PINS[3] = {FG_PIN_1, FG_PIN_2, FG_PIN_3};
const int FR_PINS[3] = {FR_PIN_1, FR_PIN_2, FR_PIN_3};
const int PWM_PINS[3] = {PWM_PIN_1, PWM_PIN_2, PWM_PIN_3};
const ledc_channel_t PWM_CHANNELS[3] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

// PCNT Handles
static pcnt_unit_handle_t pcnt_units[3];

// State variables
static int motor_directions[3] = {1, 1, 1};
static robot_odometry_t current_odom = {0};

// PID and Target variables
static float target_wheel_speeds[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_cmd_vel_time = 0;

// PID Controllers
static pid_controller_t pids[3] = {
    {.kp = 50.0f, .ki = 10.0f, .kd = 1.0f, .integral = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f},
    {.kp = 50.0f, .ki = 10.0f, .kd = 1.0f, .integral = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f},
    {.kp = 50.0f, .ki = 10.0f, .kd = 1.0f, .integral = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f}
};

// Internal function to set raw PWM
void set_motor_pwm(int id, float pwm_percent) {
    if (id < 1 || id > 3) {
        printf("incorrect motor speed set\n");
        return;
    }
    if (pwm_percent > 100.0f) {
        pwm_percent = 100.0f;
    } else if (pwm_percent < -100.0f) {
        pwm_percent = -100.0;
    }

    int index = id - 1;

    if (fabs(pwm_percent) < 1.0f) {
        ledc_set_duty(LEDC_MODE, PWM_CHANNELS[index], 0);
        ledc_update_duty(LEDC_MODE, PWM_CHANNELS[index]);
        gpio_set_level(BRA_PINS[index], 0);
        motor_directions[index] = 1;
    } else {
        gpio_set_level(BRA_PINS[index], 1);
        if (pwm_percent > 0) {
            gpio_set_level(FR_PINS[index], 0);
            motor_directions[index] = 1;
        } else {
            gpio_set_level(FR_PINS[index], 1);
            motor_directions[index] = -1;
        }

        uint32_t duty = (fabs(pwm_percent) * 1023.0f) / 100.0f;
        ledc_set_duty(LEDC_MODE, PWM_CHANNELS[index], duty);
        ledc_update_duty(LEDC_MODE, PWM_CHANNELS[index]);
    }
}

// PID Computation
static float compute_pid(pid_controller_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    
    // Anti-windup
    if (pid->integral > 50.0f) pid->integral = 50.0f;
    if (pid->integral < -50.0f) pid->integral = -50.0f;

    float derivative = (error - pid->prev_error) / dt;
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    return output;
}

// FreeRTOS task for odometry and PID
static void motor_control_task(void *arg) {
    const float dt = 0.02f; // 50Hz loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Safety timeout: stop motor if no cmd_vel for 500ms
        if ((esp_timer_get_time() - last_cmd_vel_time) > 500000) {
            target_wheel_speeds[0] = 0.0f;
            target_wheel_speeds[1] = 0.0f;
            target_wheel_speeds[2] = 0.0f;
        }

        int pulse_counts[3] = {0};
        float actual_wheel_speeds[3] = {0};

        // Read Encoders and calculate actual speeds
        for (int i = 0; i < 3; i++) {
            pcnt_unit_get_count(pcnt_units[i], &pulse_counts[i]);
            pcnt_unit_clear_count(pcnt_units[i]);

            int signed_pulses = pulse_counts[i] * motor_directions[i];
            float revolutions = (float)signed_pulses / (PULSES_PER_REV * GEAR_RATIO);
            float distance = revolutions * (2.0f * M_PI * WHEEL_RADIUS);
            actual_wheel_speeds[i] = distance / dt;
        }

        // Update Odometry (Forward kinematics)
        // V_x = (2/3) * (-V1*sin(30) - V2*sin(150) + V3*sin(270))
        // V_y = (2/3) * (V1*cos(30) + V2*cos(150) + V3*cos(270))
        // Omega = (1/(3*R)) * (V1 + V2 + V3)

        float v1 = actual_wheel_speeds[0];
        float v2 = actual_wheel_speeds[1];
        float v3 = actual_wheel_speeds[2];

        current_odom.vx = (2.0f / 3.0f) * (-0.5*v1 - 0.5*v2 + 1.0f*v3);
        current_odom.vy = (2.0f / 3.0f) * (0.866f*v1 - 0.866f*v2 + 0.0f*v3);
        current_odom.omega = (v1 + v2 + v3) / (3.0f * ROBOT_RADIUS);

        // Integrate velocities to get position
        // and transform local velocities to global frame based on current theta
        float delta_x = (current_odom.vx * cosf(current_odom.theta) - current_odom.vy * sinf(current_odom.theta)) * dt;
        float delta_y = (current_odom.vx * sinf(current_odom.theta) + current_odom.vy * cosf(current_odom.theta)) * dt;
        float delta_theta = current_odom.omega * dt;

        current_odom.x += delta_x;
        current_odom.y += delta_y;
        current_odom.theta += delta_theta;

        // Normalize theta between -PI and PI
        if (current_odom.theta > M_PI) current_odom.theta -= 2.0f * M_PI;
        if (current_odom.theta < -M_PI) current_odom.theta += 2.0f * M_PI;

        // Compute PID and apply PWM
        for (int i = 0; i < 3; i++) {
            if (fabs(target_wheel_speeds[i]) < 0.01f) {
                set_motor_pwm(i+1, 0.0f);
                pids[i].integral = 0;
            } else {
                float pwm_output = compute_pid(&pids[i], target_wheel_speeds[i], actual_wheel_speeds[i], dt);
                set_motor_pwm(i+1, pwm_output);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

void init_motors(void) {
    ESP_LOGI(TAG, "Initializing Motor GPIOs and PWM");

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
    };
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
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

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
            .level_gpio_num = -1, // Not using hardware level control, handling direct in SW
        };
        pcnt_channel_handle_t pcnt_chan = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_config, &pcnt_chan));

        // Count on rising edge
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
        
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[i]));
    }

    xTaskCreate(motor_control_task, "motor control task", 4096, NULL, 6, NULL, 0);
}

void apply_cmd_vel(float linear_x, float linear_y, float angular_z) {
    // Inverse Kinematics for 3-wheel omni (120 degrees apart)
    // Wheel 1 (front right -30 deg) Wheel 2 (front left -150 deg) Wheel 3 (back -270 deg)
    float w1 = -0.5f * linear_x + 0.866f * linear_y + ROBOT_RADIUS * angular_z;
    float w2 = -0.5f * linear_x - 0.866f * linear_y + ROBOT_RADIUS * angular_z;
    float w3 = 1.0f * linear_x + 0.0f * linear_y + ROBOT_RADIUS * angular_z;

    float max_w = fmaxf(fmaxf(fabs(w1), fabs(w2)), fabs(w3));
    if (max_w > MAX_WHEEL_SPEED) {
        w1 = (w1 / max_w) * MAX_WHEEL_SPEED;
        w2 = (w2 / max_w) * MAX_WHEEL_SPEED;
        w3 = (w3 / max_w) * MAX_WHEEL_SPEED;
    }

    target_wheel_speeds[0] = w1;
    target_wheel_speeds[1] = w2;
    target_wheel_speeds[2] = w3;

    last_cmd_vel_time = esp_timer_get_time();
}

robot_odometry_t get_odometry(void) {
    return current_odom;
}