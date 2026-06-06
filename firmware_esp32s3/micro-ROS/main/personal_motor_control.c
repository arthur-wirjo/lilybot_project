#include "personal_motor_control.h"
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

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
#define LEDC_FREQUENCY  20000

const int BRA_PINS[3] = {BRA_PIN_1, BRA_PIN_2, BRA_PIN_3};
const int FG_PINS[3] = {FG_PIN_1, FG_PIN_2, FG_PIN_3};
const int FR_PINS[3] = {FR_PIN_1, FR_PIN_2, FR_PIN_3};
const int PWM_PINS[3] = {PWM_PIN_1, PWM_PIN_2, PWM_PIN_3};
const ledc_channel_t PWM_CHANNELS[3] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

static const float PI = 3.14159265f;

// PCNT Handles
static pcnt_unit_handle_t pcnt_units[3];

// State variables
static robot_odometry_t current_odom = {0};
static int current_motor_dir[3] = {1, 1, 1};
static float filtered_speeds[3] = {0};

// PID and Target variables
static float target_wheel_speeds[3] = {0};
static float target_ramp_speeds[3] = {0};
static int64_t last_cmd_vel_time = 0;

// Feedforward Gain
static float K_linear = 0.3f;
static float K_quadratic = 0.9f;

// PID Controllers
static pid_controller_t pids[3] = {
    {.kp = 150.0f, .ki = 80.0f, .kd = 0.0f, .integral = 0, .prev_measured = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f},
    {.kp = 150.0f, .ki = 80.0f, .kd = 0.0f, .integral = 0, .prev_measured = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f},
    {.kp = 150.0f, .ki = 80.0f, .kd = 0.0f, .integral = 0, .prev_measured = 0, .prev_error = 0, .out_max = 100.0f, .out_min = -100.0f}
};

// PID output to PWM
static void set_pwm(int motor_index, float pid_output) {
    if(motor_index <= -1 || motor_index >= 3) {
        ESP_LOGI("motor_ctrl/set_pwm", "incorrect motor speed set");
        return;     
    }

    gpio_set_level(BRA_PINS[motor_index], 1);

    float duty_cycle = 0.0f;
    if (pid_output > 0.01f) {
        duty_cycle = 25.0f + fabsf(pid_output)*((67.0f - 25.0f)/100.0f);
    }
    if (duty_cycle > 67.0f) {
        duty_cycle = 67.0f;
    }

    uint32_t duty = (uint32_t)(duty_cycle*(1023.0f/100.0f));
    ledc_set_duty(LEDC_MODE, PWM_CHANNELS[motor_index], duty);
    ledc_update_duty(LEDC_MODE, PWM_CHANNELS[motor_index]);
}

static void brake_motor(int motor_index){
    ledc_set_duty(LEDC_MODE, PWM_CHANNELS[motor_index], 0);
    ledc_update_duty(LEDC_MODE, PWM_CHANNELS[motor_index]);
    gpio_set_level(BRA_PINS[motor_index], 0);
}

// calculate PID output
static float compute_pid(pid_controller_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;

    // Anti-Windup
    float max_integral = pid->out_max;
    if (pid->ki > 0.001f) {
        max_integral = pid->out_max / pid->ki;
    }
    if (pid->integral > max_integral) {
        pid->integral = max_integral;
    } else if (pid->integral < -max_integral) {
        pid->integral = -max_integral;
    }

    // practical derivative
    float derivative = -(measured - pid->prev_measured)/dt;
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }
    pid->prev_error = error;
    pid->prev_measured = measured;
    return output;
}

// FreeRTOS task (Odometry and PID)
static void motor_control_task(void *arg) {
    const float dt = 0.05f; // 50ms/20Hz loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if ((esp_timer_get_time() - last_cmd_vel_time) > 1000000) { // 1000ms
            for (int i = 0; i < 3; i++) {
                target_wheel_speeds[i] = 0.0f;
            }
        }

        // Proportional Acceleration Limiting
        float speed_diffs[3];
        float max_diff = 0.0f;

        for (int i = 0; i < 3; i++) {
            speed_diffs[i] = target_wheel_speeds[i] - target_ramp_speeds[i];
            if (fabsf(speed_diffs[i]) > max_diff) {
                max_diff = fabsf(speed_diffs[i]);
            }
        }

        float max_allowed_diff = MAX_WHEEL_ACCEL * dt;
        float scale_factor = 1.0f;
        if (max_diff > max_allowed_diff) {
            scale_factor = max_allowed_diff / max_diff;
        }

        for (int i = 0; i < 3; i++) {
            target_ramp_speeds[i] += speed_diffs[i] * scale_factor;
        }

        // Encoder
        int pulse_counts[3] = {0};
        float actual_wheel_speeds[3] = {0};
        for (int i = 0; i < 3; i++) {
            pcnt_unit_get_count(pcnt_units[i], &pulse_counts[i]);
            pcnt_unit_clear_count(pcnt_units[i]);
            
            int signed_pulses = pulse_counts[i] * current_motor_dir[i];
            float revolutions = (float)signed_pulses/(PULSES_PER_REV*GEAR_RATIO);
            float distance = revolutions * (2.0f * PI * WHEEL_RADIUS);
            float raw_speed = distance/dt;

            filtered_speeds[i] = (0.7f * filtered_speeds[i]) + (0.3f * raw_speed);
            actual_wheel_speeds[i] = filtered_speeds[i];
        }

        // Update Odometry
        float v1 = actual_wheel_speeds[0];
        float v2 = actual_wheel_speeds[1];
        float v3 = actual_wheel_speeds[2];

        current_odom.vx = (2.0f / 3.0f) * (0.866666f*v1 - 0.866666f*v2 + 0.0f*v3);
        current_odom.vy = (2.0f / 3.0f) * (0.5f*v1 + 0.5f*v2 - 1.0f*v3);
        current_odom.omega = (v1 + v2 + v3) / (3.0f * ROBOT_RADIUS);

        float delta_theta = current_odom.omega * dt;
        float dx_local;
        float dy_local;

        if (fabsf(current_odom.omega) > 0.001f) {
            dx_local = (current_odom.vx * sinf(delta_theta) + current_odom.vy * (cosf(delta_theta) - 1.0f)) / current_odom.omega;
            dy_local = (current_odom.vx * (1.0f - cosf(delta_theta)) + current_odom.vy * sinf(delta_theta)) / current_odom.omega;
        } else {
            dx_local = current_odom.vx * dt;
            dy_local = current_odom.vy * dt;
        }

        float global_dx = dx_local * cosf(current_odom.theta) - dy_local * sinf(current_odom.theta);
        float global_dy = dx_local * sinf(current_odom.theta) + dy_local * cosf(current_odom.theta);

        current_odom.x += global_dx;
        current_odom.y += global_dy;
        current_odom.theta += delta_theta;

        // Normalize theta between -PI and PI
        while (current_odom.theta > PI) current_odom.theta -= 2.0f * PI;
        while (current_odom.theta < -PI) current_odom.theta += 2.0f * PI;

        // Compute PID and Apply PWM
        for (int i = 0; i < 3; i++) {
            if (fabsf(target_ramp_speeds[i]) < 0.001f && fabsf(target_wheel_speeds[i]) < 0.001f) {
                brake_motor(i);
                pids[i].integral = 0.0f;
                pids[i].prev_measured = fabsf(actual_wheel_speeds[i]);
            } else {
                if (target_ramp_speeds[i] >= 0) {
                    gpio_set_level(FR_PINS[i], 0);
                    current_motor_dir[i] = 1;
                } else {
                    gpio_set_level(FR_PINS[i], 1);
                    current_motor_dir[i] = -1;
                }
                float abs_target = fabsf(target_ramp_speeds[i]);
                float abs_measured = fabsf(actual_wheel_speeds[i]);

                // PID
                float pid_output = compute_pid(&pids[i], abs_target, abs_measured, dt);
                // 2nd-Order feedforward control
                float v_norm = (abs_target / MAX_WHEEL_SPEED);
                float feedforward = ((K_linear * v_norm) + (K_quadratic * v_norm*v_norm)) * 100.0f;
                
                // total output
                float total_output = pid_output + feedforward;

                if (total_output < 0.0f) {
                    total_output = 0.0f;
                }
                printf("id: %d | tar: %.2f | mes: %.2f | feed: %.2f | pid: %.2f | tot: %.2f\n", i, abs_target, abs_measured, feedforward, pid_output, total_output);
                set_pwm(i, total_output);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void init_motors(void) {
    ESP_LOGI("motor_ctrl/init_motors", "Initializing Motor GPIOs and PWM");

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

        // Count on both rising and falling edge
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[i]));
    }

    xTaskCreatePinnedToCore(motor_control_task, "motor control task", 4096, NULL, 6, NULL, 0);
}

void apply_cmd_vel(float linear_x, float linear_y, float angular_z) {
    // Inverse Kinematics for 3-wheel omni (120 degrees apart)
    // Wheel 1 (front right -30 deg) Wheel 2 (front left -150 deg) Wheel 3 (back -270 deg)
    float w1 = 0.866666f * linear_x + 0.5f * linear_y + ROBOT_RADIUS * angular_z;
    float w2 = -0.866666f * linear_x + 0.5f * linear_y + ROBOT_RADIUS * angular_z;
    float w3 = 0.0f * linear_x + -1.0f * linear_y + ROBOT_RADIUS * angular_z;

    float max_w = fmaxf(fmaxf(fabsf(w1), fabsf(w2)), fabsf(w3));
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