// Documentation notes for self:
// Currently the imu data topic is not being published and idk why
// Initially it worked but now it doesn't and i suspect its because
// I tried to modify i2c_scanner to call a function instead and perhaps
// It messed up the ISR in esp perhaps 
// Doesn't seem like a big deal because gemini suggested
// Passing the bus handle as an argument to the micro-ROS task
// which makes more sense and hopefully will fix the problem
// if not then goodluck lmao
// i think its worth experimenting with differnt codes
// try use github

#include <string.h>
#include <stdio.h>
#include <unistd.h>
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

#include "mpu9250.h"

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

// Brake Pins
#define BRAKE_PIN_1 42
#define BRAKE_PIN_2 15
#define BRAKE_PIN_3 16  

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

// Brake Initialization
void init_brakes() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BRAKE_PIN_1) | (1ULL << BRAKE_PIN_2) | (1ULL << BRAKE_PIN_3);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(BRAKE_PIN_1, 0);
    gpio_set_level(BRAKE_PIN_2, 0);
    gpio_set_level(BRAKE_PIN_3, 0);

    printf("Brakes Engaged\n");
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

    // Create Node & Publisher
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_s3_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // Allocate Frame ID
    imu_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    sprintf(imu_msg.header.frame_id.data, "imu_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = 20;

    // Main Loop
    while(1) {
        if (mpu9250_update(&mpu_device) == ESP_OK) {
            // Convert Raw Data to SI Units
            imu_msg.linear_acceleration.x = (float)mpu_device.accel.x / 16384.0 * 9.81;
            imu_msg.linear_acceleration.y = (float)mpu_device.accel.y / 16384.0 * 9.81;
            imu_msg.linear_acceleration.z = (float)mpu_device.accel.z / 16384.0 * 9.81;

            imu_msg.angular_velocity.x = (float)mpu_device.gyro.x / 131.0 * 0.01745;
            imu_msg.angular_velocity.y = (float)mpu_device.gyro.y / 131.0 * 0.01745;
            imu_msg.angular_velocity.z = (float)mpu_device.gyro.z / 131.0 * 0.01745;
        } else {
            printf("I2C Read Error\n");
        }

        int64_t time_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = time_ns / 1000000000;
        imu_msg.header.stamp.nanosec = time_ns % 1000000000;

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    // Currently task will never end but useful for future
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void) {
    // Engage Brakes
    init_brakes();

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