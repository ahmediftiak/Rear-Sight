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
    for (uint8_t addr = 1; addr < 127; ++addr) {
        uint8_t temp;
        if (i2c.read(addr, &temp, 1) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
        }
    }

    // TODO: Initialize TF-Luna sensors with I2C bus
    // TODO: Read distance and process data
}