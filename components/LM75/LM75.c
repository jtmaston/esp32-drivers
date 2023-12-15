#include <stdio.h>
#include "LM75.h"

int16_t readTemp(const struct i2cDevice dev, const uint16_t timeout) {      // returns a fixed-point value, 1e2 factor
    uint16_t data = readDoublei2cReg(dev, 0, timeout);
    if(data == -1000)
        return -1000;

    switch(data >> 15)
    {
        case 0:
            data = data >> 5;
        return (int) (data * 0.125 * 100.0f);
        case 1:
            data = data >> 5;
        return (int32_t)(((~data + 1) & 0b0000011111111111) * -(0.125) ) * 100.0f;
        default:
            break;
    }

    return 0;
}