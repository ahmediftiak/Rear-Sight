// /main/main.cpp

#include "TFLuna.hpp"
#include "I2CBus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "RearSight"

// Define I2C pins for TF-Luna sensors
#define I2C_SDA_PIN GPIO_NUM_11
#define I2C_SCL_PIN GPIO_NUM_12

void scanI2CBus(i2c_port_t port) {
    ESP_LOGI("I2C_SCAN", "Starting I2C scan on port %d...", port);
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Device found at address 0x%02X", addr);
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGD("I2C_SCAN", "No response at 0x%02X: %s", addr, esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Optional: Avoid overwhelming bus
    }
    ESP_LOGI("I2C_SCAN", "I2C scan complete.");
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting RearSight system...");

    // Initialize I2C bus on defined GPIO pins
    I2CBus i2c(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    esp_err_t ret = i2c.init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C bus initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    // Scan for I2C devices on the bus
    scanI2CBus(I2C_NUM_0);

    // TODO: Initialize TF-Luna sensors with I2C bus
    // TODO: Read distance and process data
}