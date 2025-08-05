#ifndef I2CBUS_HPP
#define I2CBUS_HPP

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"


class I2CBus {
public:
    /**
     * @brief Constructor for I2CBus
     * 
     * @param port I2C port number (I2C_NUM_0, I2C_NUM_1, etc.)
     * @param sda GPIO pin number for SDA
     * @param scl GPIO pin number for SCL
     * @param clk_speed Clock speed in Hz (default is 100000 for 100kHz)
     */
    I2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed = 100000);
    // @brief Destructor for I2CBus
    ~I2CBus();

    /**
     * @brief Initialize the I2C bus
     * 
     * @return esp_err_t ESP_OK on success, error code on failure
     */    
    esp_err_t init();
    esp_err_t deinit();
    /**
     * @brief Write data to a specific I2C address
     * 
     * @param address I2C address to write to
     * @param data Pointer to the data buffer to write
     * @param length Length of the data buffer
     * @return esp_err_t ESP_OK on success, error code on failure
     */
    esp_err_t writeRead(uint8_t address, const uint8_t* write_data, size_t write_length, uint8_t* read_data, size_t read_length);
    /**
     * @brief Write data to a specific I2C address
     * 
     * @param address I2C address to write to
     * @param data Pointer to the data buffer to write
     * @param length Length of the data buffer
     * @return esp_err_t ESP_OK on success, error code on failure
     */
    esp_err_t writeBytes(uint8_t address, const uint8_t* data, size_t length);
    /**
     * @brief Read data from a specific I2C address
     * 
     * @param address I2C address to read from
     * @param data Pointer to the buffer to store read data
     * @param length Length of the data buffer
     * @return esp_err_t ESP_OK on success, error code on failure
     */
    esp_err_t readBytes(uint8_t address, uint8_t* data, size_t length);

private:
    /**
     * @brief Configure the I2C bus with the specified parameters
     * 
     * @return esp_err_t ESP_OK on success, error code on failure
     */
    i2c_port_t port_;      // I2C port (e.g. I2C_NUM_0)
    gpio_num_t sda_;       // SDA GPIO pin
    gpio_num_t scl_;       // SCL GPIO pin
    uint32_t freq_;        // I2C clock speed
};
#endif // I2CBus_HPP