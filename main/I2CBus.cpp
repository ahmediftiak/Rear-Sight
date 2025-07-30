#include "I2CBus.hpp"
#include "freertos/FreeRTOS.h"

I2CBus::I2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq)
    : port_(port), sda_(sda), scl_(scl), freq_(freq) {}

I2CBus::~I2CBus() {
    deinit();
}
// @brief Initialize the I2C bus
esp_err_t I2CBus::init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_;
    conf.scl_io_num = scl_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq_;
    ESP_ERROR_CHECK(i2c_param_config(port_, &conf));
    return i2c_driver_install(port_, conf.mode, 0, 0, 0);
}
// @brief Deinitialize the I2C bus
esp_err_t I2CBus::deinit() {
    return i2c_driver_delete(port_);
}
// @brief Write and read data to a specific I2C address
esp_err_t I2CBus::writeRead(uint8_t address, const uint8_t* write_data, size_t write_length, uint8_t* read_data, size_t read_length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_data, write_length, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if (read_length > 0) {
        i2c_master_read(cmd, read_data, read_length - 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &read_data[read_length - 1], I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
// @brief Write data to a specific I2C address
esp_err_t I2CBus::write(uint8_t address, const uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, length, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
// @brief Read data from a specific I2C address
esp_err_t I2CBus::read(uint8_t address, uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if (length > 0) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[length - 1], I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}