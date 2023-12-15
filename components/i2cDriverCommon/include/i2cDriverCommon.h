#ifndef I2C_DRIVER_COMMON_H
#define I2C_DRIVER_COMMON_H
#include <driver/i2c.h>
#include <stdbool.h>

// #define DEBUG    // -> uncomment to enable debug mode, with extra prints and idk

struct i2cDevice{
    i2c_port_t portNum;
    uint16_t address;
};

void initBus(uint8_t scl, uint8_t sda, i2c_port_t num,
           uint32_t freq, bool pullUp);

uint16_t readDoublei2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout);
uint8_t readSinglei2cReg(struct i2cDevice dev, uint8_t reg, uint16_t timeout);

void writeDoublei2cReg(struct i2cDevice dev, uint8_t reg, uint8_t data, uint8_t data2);
void writeSinglei2cReg(struct i2cDevice dev, uint8_t reg, uint8_t data);


#endif