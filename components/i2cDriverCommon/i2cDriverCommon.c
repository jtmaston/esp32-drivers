#include <stdio.h>
#include "i2cDriverCommon.h"

uint16_t readDoublei2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout) {
#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.lock();
#endif
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                               // datasheet page 21
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev.address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);

    uint8_t data1, data2;
    i2c_master_read_byte(cmd, &data1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data2, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT)
        return -1000;
    uint16_t data = data1 << 8 | data2;

#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.unlock();
#endif
    return data;
}

uint8_t readSinglei2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, dev.address << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(dev.portNum, cmd, timeout / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

void writeSinglei2cReg(struct i2cDevice dev, uint8_t reg, uint8_t data1) {

#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.lock();
#endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                   // i2c commands as described by AT24CM02 datasheet,
    i2c_master_start(cmd);                                          // page 17
    i2c_master_write_byte(cmd, (dev.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data1, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev.portNum, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.unlock();
#endif
}

void writeDoublei2cReg(struct i2cDevice dev, uint8_t reg, uint8_t data1, uint8_t data2) {
#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.lock();
#endif
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                   // i2c commands as described by AT24CM02 datasheet,
    i2c_master_start(cmd);                                          // page 17
    i2c_master_write_byte(cmd, (dev.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data1, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data2, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(dev.portNum, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
#ifdef M_USE_MUTEX
    M_MUTEX_PROTECTOR.unlock();
#endif
}

void initBus(const uint8_t scl, const uint8_t sda, const i2c_port_t num,
           const uint32_t freq, const bool pullUp)
{
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
}