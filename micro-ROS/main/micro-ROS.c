// Documentation notes for self:
// make the micro-ROS motion control work

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h" 

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "mpu9250.h"
#include "personal_motor_control.h"

// UART Configuration
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define TX_PIN             17 
#define RX_PIN             18
#define BUF_SIZE           1024

// I2C Configuration
#define I2C_MASTER_SCL_IO  9 
#define I2C_MASTER_SDA_IO  8  
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 50000 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// cmd_vel callback
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    apply_cmd_vel(msg->linear.x, msg->linear.y, msg->angular.z);
}

// Helper function for Euler Yaw to Quaternion
void euler_to_quat(float yaw, double* qx, double* qy, double* qz, double* qw) {
    *qx = 0.0;
    *qy = 0.0;
    *qz = sin(yaw * 0.5);
    *qw = cos(yaw * 0.5);
}

// Custom Transport Implementation
bool cubemx_transport_open(struct uxrCustomTransport * transport) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(UART_PORT_NUM, &uart_config) != ESP_OK) return false;
    if (uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) return false;
    return true;
}
bool cubemx_transport_close(struct uxrCustomTransport * transport) {
    return uart_driver_delete(UART_PORT_NUM) == ESP_OK;
}
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    return uart_write_bytes(UART_PORT_NUM, (const char*)buf, len);
}
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    int rxBytes = uart_read_bytes(UART_PORT_NUM, buf, len, pdMS_TO_TICKS(timeout));
    return (rxBytes < 0) ? 0 : rxBytes;
}

// Micro-ROS Task
void micro_ros_task(void * arg) {
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize Transport
    rmw_uros_set_custom_transport(
        true, NULL,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read
    );

    // Initialize IMU
    mpu9250_t mpu_device;
    mpu9250_config_t mpu_config = {0};
    printf("Initializing MPU...\n");
    if (mpu9250_begin(&mpu_device, mpu_config, 0x68, bus_handle) == ESP_OK) {
        printf("MPU9250 Ready.\n");
    } else {
        printf("MPU9250 Init Failed.\n");
    }

    // Wait for Agent
    printf("Waiting for agent...\n");
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("Agent connected!\n");

    // Create Node
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_s3_node", "", &support));

    // Create Publishers
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // Create Subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Create Executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    // Allocate Frame ID / String
    imu_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    sprintf(imu_msg.header.frame_id.data, "imu_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = 20;

    odom_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    sprintf(odom_msg.header.frame_id.data, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.header.frame_id.capacity = 20;

    odom_msg.child_frame_id.data = (char*)malloc(20 * sizeof(char));
    sprintf(odom_msg.child_frame_id.data, "base_link");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
    odom_msg.child_frame_id.capacity = 20;

    // Main Loop
    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Fetch odometry
        robot_odometry_t odom = get_odometry();

        int64_t time_ns = rmw_uros_epoch_nanos();

        // Populate odometry msg
        odom_msg.header.stamp.sec = time_ns / 1000000000;
        odom_msg.header.stamp.nanosec = time_ns % 1000000000;

        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.position.y = odom.y;
        odom_msg.pose.pose.position.z = 0.0;

        double qx, qy, qz, qw;
        euler_to_quat(odom.theta, &qx, &qy, &qz, &qw);
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;

        odom_msg.twist.twist.linear.x = odom.vx;
        odom_msg.twist.twist.linear.y = odom.vy;
        odom_msg.twist.twist.angular.z = odom.omega;

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

        // Populate IMU msg
        if (mpu9250_update(&mpu_device) == ESP_OK) {
            imu_msg.header.stamp.sec = time_ns / 1000000000;
            imu_msg.header.stamp.nanosec = time_ns % 1000000000;

            // Convert Raw Data to SI Units
            imu_msg.linear_acceleration.x = (float)mpu_device.accel.x / 16384.0 * 9.81;
            imu_msg.linear_acceleration.y = (float)mpu_device.accel.y / 16384.0 * 9.81;
            imu_msg.linear_acceleration.z = (float)mpu_device.accel.z / 16384.0 * 9.81;

            imu_msg.angular_velocity.x = (float)mpu_device.gyro.x / 131.0 * 0.01745;
            imu_msg.angular_velocity.y = (float)mpu_device.gyro.y / 131.0 * 0.01745;
            imu_msg.angular_velocity.z = (float)mpu_device.gyro.z / 131.0 * 0.01745;

            RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        } else {
            printf("I2C Read Error\n");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
    // Currently task will never end but useful for future
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void) {
    // Engage Brakes
    init_motors();

    // Initialize I2C on Core 0 directly in app main 
    // Because doing it as a task results in I2C not initializing properly
    // Most likely because of the Micro-ROS
    printf("Initializing I2C on Core 0...\n");
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    vTaskDelay(pdMS_TO_TICKS(200)); // Allow voltage to stabilize

    // Probing I2C
    printf("Probing 0x68...\n");
    esp_err_t probe_ret = i2c_master_probe(bus_handle, 0x68, 50);
    
    if (probe_ret == ESP_OK) {
        printf("Probe Success!\n");
    } else {
        printf("Probe Failed (Error %d). Check wires.\n", probe_ret);
    }

    // Micro-ROS task
    xTaskCreatePinnedToCore(micro_ros_task, "uros_task", 8192, (void*)bus_handle, 5, NULL, 1);
}