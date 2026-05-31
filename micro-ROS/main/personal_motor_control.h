#pragma once

#include <stdint.h>

#define ROBOT_RADIUS 0.135f // 135mm
#define WHEEL_RADIUS 0.0375f // 37.5mm
#define MAX_WHEEL_SPEED 0.5f // 0.5m/s 
#define MAX_WHEEL_ACCEL 0.5f // 0.5m/s^2
#define PULSES_PER_REV 12.0f // 6 pulses per rev but count rising and falling edge
#define GEAR_RATIO 50.0f // 1:50 Reduction

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_measured;
    float out_max;
    float out_min;
} pid_controller_t;

typedef struct {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
} robot_odometry_t;

void init_motors(void);
void apply_cmd_vel(float linear_x, float linear_y, float angular_z);
robot_odometry_t get_odometry(void);