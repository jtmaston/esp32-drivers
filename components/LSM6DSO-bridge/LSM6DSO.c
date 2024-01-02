#include "LSM6DSO.h"

int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
    struct i2cDevice* dev = (struct i2cDevice*) handle;
    writeMultipleI2cReg(*dev, Reg, 100, Bufp, len);

    return 0;
}
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
    struct i2cDevice* dev = (struct i2cDevice*) handle;
    readMultipleI2cReg(*dev, Reg, 100, Bufp, len);
    return 0;
}