#ifndef I2C_DRIVER_LSM6DSO_H
#define I2C_DRIVER_LSM6DSO_H
#include <driver/i2c.h>
#include <esp_log.h>
#include "i2cDriverCommon.h"
#define M_LSM6DSO_TAG "LSM6DSO-bridge"

struct triplet{
    float x, y, z;
};

esp_err_t initLSM6DSO(struct i2cDevice);
esp_err_t getLinearAccelerationsLSM6DSO(struct i2cDevice dev, struct triplet* data);      // todo: move away from float
esp_err_t getRotationalAccelerationsLSM6DSO(struct i2cDevice dev, struct triplet* data);
uint16_t getFIFOSampleSize(struct i2cDevice dev);
uint32_t getFIFOSample(struct i2cDevice dev);

bool checkDataReadyLSM6DSO(struct i2cDevice);

#endif