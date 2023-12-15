#include <stdio.h>
#include "i2cDriverCommon.h"
#include "LM75.h"

void app_main(void)
{
    initBus(2, 1, I2C_NUM_0, 125000, false);
    struct i2cDevice tempSensor = {I2C_NUM_0, 0x49};
    while(true) {
        printf("%f\n", readTemp(tempSensor, 100) / 100.0f);
        fflush(stdout);
    }
}
