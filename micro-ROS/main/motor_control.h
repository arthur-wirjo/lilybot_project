#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// Kinematics geometry
#define ROBOT_RADIUS 0.135f 
#define WHEEL_RADIUS 0.0375f 
#define MAX_WHEEL_SPEED 100.0f // PLEASE ADJUST LATER

// Encoder configuration
#define PULSES_PER_REV 6.0f // 6 pulse/rev
#define GEAR_RATIO 50.0f // 1:50 Reduction Ratio

typedef struct{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float setpoint;
    float current_val;
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

void set_motor_speed(int motor_id, int speed);

void apply_cmd_vel(float linear_x, float linear_y, float angular_z);

float compute_pid(pid_controller_t *pid, float setpoint, float measured, float dt);

void update_odometry(float dt);
robot_odometry_t get_odometry(void);

#endif // MOTOR_CONTROL_H