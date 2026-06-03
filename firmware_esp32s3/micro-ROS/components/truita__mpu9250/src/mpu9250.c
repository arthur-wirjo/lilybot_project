#include "mpu9250.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> // For memset

static const char *TAG = "mpu9250";

// --- Registers ---
#define MPU9250_WHO_AM_I      0x75
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_SMPLRT_DIV    0x19
#define MPU9250_CONFIG        0x1A
#define MPU9250_GYRO_CONFIG   0x1B
#define MPU9250_ACCEL_CONFIG  0x1C
#define MPU9250_ACCEL_XOUT_H  0x3B

// --- Helper: Write Register ---
static esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(handle, write_buf, sizeof(write_buf), -1);
}

// --- Helper: Read Registers ---
static esp_err_t read_regs(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, -1);
}

int mpu9250_begin(mpu9250_t *mpu, const mpu9250_config_t config, int address,
                  i2c_master_bus_handle_t i2c_bus_handle) {
    
    ESP_LOGI(TAG, "Initializing MPU9250 at 0x%02X...", address);

    // 1. Add Device to Bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = address,
        .scl_speed_hz = 100000, // Slow down to 100kHz for stability
    };
    
    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &mpu->_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus");
        return err;
    }

    // 2. Check WHO_AM_I
    uint8_t who_am_i;
    err = read_regs(mpu->_handle, MPU9250_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate (NACK). Check wires.");
        return err;
    }
    ESP_LOGI(TAG, "Device ID: 0x%02X", who_am_i);

    // 3. Reset the Sensor (Critical Step)
    write_reg(mpu->_handle, MPU9250_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset

    // 4. Wake Up (Clear Sleep Bit) and Select Auto Clock
    // Bit 0-2: CLKSEL (1 = Auto), Bit 6: SLEEP (0 = Wake)
    err = write_reg(mpu->_handle, MPU9250_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 5. Configure Sample Rate & DLPF
    // CONFIG: DLPF_CFG = 3 (41Hz Bandwidth, ~5ms delay) - Good for balancing robots
    write_reg(mpu->_handle, MPU9250_CONFIG, 0x03);
    
    // SMPLRT_DIV: Sample Rate = 1kHz / (1 + 4) = 200Hz
    write_reg(mpu->_handle, MPU9250_SMPLRT_DIV, 0x04);

    // 6. Configure Gyro Range (250 dps)
    // Bit 3-4: FS_SEL (00 = 250dps)
    write_reg(mpu->_handle, MPU9250_GYRO_CONFIG, 0x00);

    // 7. Configure Accel Range (2G)
    // Bit 3-4: AFS_SEL (00 = 2G)
    write_reg(mpu->_handle, MPU9250_ACCEL_CONFIG, 0x00);

    ESP_LOGI(TAG, "MPU9250 Configured Successfully");
    return ESP_OK;
}

int mpu9250_update(mpu9250_t *mpu) {
    uint8_t raw_data[14];
    
    // Burst Read 14 Bytes: Accel (6) + Temp (2) + Gyro (6)
    esp_err_t err = read_regs(mpu->_handle, MPU9250_ACCEL_XOUT_H, raw_data, 14);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return err;
    }

    // Parse Data (Big Endian: High Byte first)
    mpu->accel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    mpu->accel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    mpu->accel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Temp is bytes 6-7 (Skipping)

    mpu->gyro.x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    mpu->gyro.y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    mpu->gyro.z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return ESP_OK;
}