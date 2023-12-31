#include <stdio.h>
#include "LM75.h"

/*!
 * Reads temperature from a LM75 connected device
 * @param dev I2c device object bound to the sensor
 * @param timeout Timeout in milliseconds
 * @return Temperature, as a fixed-point value, multiplied by 1e2.
 */
int16_t readTempLM75(struct i2cDevice dev, uint16_t timeout) {      // returns a fixed-point value, 1e2 factor
    uint16_t data;
    readDoubleI2cReg(dev, 0, timeout, &data);   // todo: handle timeout errors

    switch(data >> 15)
    {
        case 0:
            data = data >> 5;
        return (int16_t) (data * 0.125 * 100.0f);
        case 1:
            data = data >> 5;
        return (int16_t)(((~data + 1) & 0b0000011111111111) * -(0.125) ) * 100.0f;
        default:
            break;
    }

    return 0;
}