#ifndef I2C_DRIVER_LM75_H
#define I2C_DRIVER_LM75_H
#include "i2cDriverCommon.h"

int16_t readTemp(struct i2cDevice dev, uint16_t timeout);

#endif