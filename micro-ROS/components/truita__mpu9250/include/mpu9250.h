#pragma once

#include "driver/i2c_master.h"
#include <stdint.h>

// Configuration Struct (We kept this simple)
typedef struct {
    bool accel_enabled;
    bool gyro_enabled;
    bool temp_enabled;
    int accel_filter_level;
    int gyro_temp_filter_level;
} mpu9250_config_t;

// Vector Struct for X, Y, Z data
struct mpu9250_vector3 {
    int16_t x;
    int16_t y;
    int16_t z;
};

// Main Device Struct
typedef struct {
    mpu9250_config_t config;
    struct mpu9250_vector3 accel;
    struct mpu9250_vector3 gyro;
    float temp;
    i2c_master_dev_handle_t _handle; // The ESP-IDF I2C Device Handle
} mpu9250_t;

// Function Prototypes
/**
 * @brief Initialize the MPU9250 sensor
 * 
 * @param mpu Pointer to the device struct
 * @param config Configuration settings (ignored in bare metal driver, but kept for compatibility)
 * @param address I2C Address (usually 0x68)
 * @param i2c_bus_handle The ESP-IDF Bus Handle
 * @return 0 (ESP_OK) on success, or error code
 */
int mpu9250_begin(mpu9250_t *mpu, const mpu9250_config_t config, int address,
                  i2c_master_bus_handle_t i2c_bus_handle);

/**
 * @brief Read the latest data from the sensor
 * 
 * @param mpu Pointer to the device struct
 * @return 0 (ESP_OK) on success, or error code
 */
int mpu9250_update(mpu9250_t *mpu);