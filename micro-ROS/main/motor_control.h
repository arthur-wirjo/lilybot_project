#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// Kinematics geometry
#define ROBOT_RADIUS 0.135f // 135mm
#define WHEEL_RADIUS 0.0375f // 37.5mm
#define MAX_WHEEL_SPEED 0.9f // 0.9m/s (actual max ~1.0m/s)

// Encoder configuration
#define PULSES_PER_REV 12.0f // 6 pulse/rev
#define GEAR_RATIO 50.0f // 1:50 Reduction Ratio

// Acceleration and speed limit config
#define MAX_ACCEL 1.5f
#define SPEED_FILTER_ALPHA 0.3f

typedef struct{
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
    float x; // in meters
    float y; // in meters
    float theta; // in radians
    float vx; // in m/s
    float vy; // in m/s
    float omega; // in rad/s
} robot_odometry_t;

void init_motors(void);
void apply_cmd_vel(float linear_x, float linear_y, float angular_z);
robot_odometry_t get_odometry(void);

#endif // MOTOR_CONTROL_H