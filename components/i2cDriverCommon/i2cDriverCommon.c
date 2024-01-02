#include <stdio.h>
#include "i2cDriverCommon.h"

/*!
 * Semaphore used in order to ensure that calls to the i2c interface are thread-safe (maybe)
 */
SemaphoreHandle_t I2cSemaphore;


/*!
 * Reads data from two adjacent registers (requires peripheral to support auto-increment)
 * @param dev device to be accessed
 * @param reg register from which data should be read (base address from which we increment)
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data word
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t readDoubleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint16_t *data) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {
        uint8_t data1, data2;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &data1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data2, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        const esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        *data = data1 << 8 | data2;
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;             // have the user handle timeout

}

/*!
 * Reads data from a single i2c register
 * @param dev device to be accessed
 * @param reg register from which data should be read
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data byte
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t readSingleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t *data) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {
        const i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        const esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;
}

/*!
 * Reads data from an arbitrary number of adjacent i2c registers (requires peripheral to support auto-increment)
 * @param dev device to be accessed
 * @param reg register base address from which data should be read
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data byte array (warning: no built-in checks for buffer size)
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t readMultipleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t *data, uint8_t numBytes) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {
        const i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_READ, true);
        if (numBytes > 1)
            i2c_master_read(cmd, data, numBytes - 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        const esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;
}

/*!
 * Writes data to a single i2c register
 * @param dev device to be accessed
 * @param reg register to which data should be written
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data byte
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t writeSingleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t data) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;
}

/*!
 * Writes word to two adjacent i2c registers (requires peripheral to support auto-increment)
 * @param dev device to be accessed
 * @param reg register base address to which data should be written
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data byte
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t writeDoubleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t data, uint8_t data2) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_write_byte(cmd, data2, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;
}

/*!
 * Writes word to two adjacent i2c registers (requires peripheral to support auto-increment)
 * @param dev device to be accessed
 * @param reg register base address to which data should be written
 * @param timeout timeout, in milliseconds
 * @param data pointer to the data byte
 * @return [TEMPORARY] returns the result of the command to the bus.
 */
esp_err_t writeMultipleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, const uint8_t* data, uint8_t numBytes) {
    if (xSemaphoreTake(I2cSemaphore, portMAX_DELAY)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);

        for(uint_fast8_t i = 0 ; i < numBytes;i ++)
        {
            i2c_master_write_byte(cmd, data[i], true);
        }

        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(I2cSemaphore);
        return ret;
    } else
        return ESP_ERR_TIMEOUT;
}


/*!
 * Initializes the i2c bus. Should be called before using any driver functions!
 * @param scl SCL pin
 * @param sda SDA pin
 * @param num Port number to be used
 * @param freq Frequency (in hz, eg 4e5)
 * @param pullUp Enable or disable builtin pull-ups
 */
void initBus(const uint8_t scl, const uint8_t sda, const i2c_port_t num,
             const uint32_t freq, const bool pullUp) {
    const i2c_config_t config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .sda_pullup_en = pullUp,
            .scl_pullup_en = pullUp,
            .master.clk_speed = freq
    };

    i2c_param_config((i2c_port_t) num, &config);
    i2c_driver_install((i2c_port_t) num, config.mode, 0, 0, 0);

    I2cSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(I2cSemaphore);
}