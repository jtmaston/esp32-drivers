#ifndef I2C_DRIVER_LSM6DSO_H
#define I2C_DRIVER_LSM6DSO_H
#include <driver/i2c.h>
#include <esp_log.h>
#include "i2cDriverCommon.h"
#define M_LSM6DSO_TAG "LSM6DSO-bridge"
#include <lsm6dso_reg.h>
#include <i2cDriverCommon.h>

struct triplet{
    float x, y, z;
};

int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

#endif