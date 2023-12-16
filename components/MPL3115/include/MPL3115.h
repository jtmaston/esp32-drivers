#ifndef I2C_DRIVER_MPL3115_H
#define I2C_DRIVER_MPL3115_H
#include <driver/i2c.h>
#include <esp_log.h>
#include "i2cDriverCommon.h"
#define M_MPL3115_TAG "MPL3115"

esp_err_t initMPL3115(struct i2cDevice);
float getPressureBarMPL3115(struct i2cDevice);      // todo: move away from float
float getTemperatureCMPL3115(struct i2cDevice);
bool checkDataReadyMPL3115(struct i2cDevice);

#endif