#ifndef I2C_DRIVER_COMMON_H
#define I2C_DRIVER_COMMON_H
#include <driver/i2c.h>
#include <stdbool.h>

// #define DEBUG    // -> uncomment to enable debug mode, with extra prints and idk

#include "freertos/semphr.h"
extern SemaphoreHandle_t I2cSemaphore;

struct i2cDevice{
    i2c_port_t portNum;
    uint16_t address;
};

void initBus(uint8_t scl, uint8_t sda, i2c_port_t num,
           uint32_t freq, bool pullUp);

esp_err_t readDoubleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint16_t* data);
esp_err_t readSingleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t* data);
esp_err_t readMultipleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t* data, uint8_t numBytes);

esp_err_t writeDoubleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t data, uint8_t data2);
esp_err_t writeSingleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, uint8_t data);
esp_err_t writeMultipleI2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout, const uint8_t* data, uint8_t numBytes);


#endif