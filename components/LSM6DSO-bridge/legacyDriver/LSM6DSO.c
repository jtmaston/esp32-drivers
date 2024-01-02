#include <stdio.h>
#include "LSM6DSO.h"

/*!
 * LSM6DSO-bridge specific registers
 */


enum registers {
    FUNC_CFG_ACCESS,
    PIN_CTRL,
    EMB_FUNC_EN_B = 0x05,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2,
    FIFO_CTRL3,
    FIFO_CTRL4,
    COUNTER_BDR_REG1,
    COUNTER_BDR_REG2,
    INT1_CTRL,
    INT2_CTRL,
    WHO_AM_I,
    CTRL1_XL,
    CTRL2_G,
    CTRL3_C,
    CTRL4_C,
    CTRL5_C,
    CTRL6_C,
    CTRL7_G,
    CTRL8_XL,
    CTRL9_XL,
    CTRL10_C,
    ALL_INT_SRC,
    WAKE_UP_SRC,
    TAP_SRC,
    D6D_SRC,
    STATUS_REG,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H,
    OUTX_L_G,
    OUTX_H_G,
    OUTY_L_G,
    OUTY_H_G,
    OUTZ_L_G,
    OUTZ_H_G,
    OUTX_L_A,
    OUTX_H_A,
    OUTY_L_A,
    OUTY_H_A,
    OUTZ_L_A,
    OUTZ_H_A,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE,
    FSM_STATUS_B_MAINPAGE,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1,
    FIFO_STATUS2,
    TIMESTAMP0 = 40,
    TIMESTAMP1,
    TIMESTAMP2,
    TIMESTAMP3,
    TAP_CFG0 = 56,
    TAP_CFG1,
    TAP_CFG2,
    TAP_THS_6D,
    INT_DUR2,
    WAKE_UP_THS,
    WAKE_UP_DUR,
    FREE_FALL,
    MD1_CFG,
    MD2_CFG,
    I3C_BUS_AVB = 62,
    INTERNAL_FREQ_FINE,
    INT_OIS = 0x6F,
    CTRL1_OIS,
    CTRL2_OIS,
    CTRL3_OIS,
    X_OFS_USR,
    Y_OFS_USR,
    Z_OFS_USR,
    FIFO_DATA_OUT_TAG = 78,
    FIFO_DATA_OUT_X_L,
    FIFO_DATA_OUT_X_H,
    FIFO_DATA_OUT_Y_L,
    FIFO_DATA_OUT_Y_H,
    FIFO_DATA_OUT_Z_L
};


/*!
 *  @brief Initializes the sensor
 *  @param dev device to which the barometer binds
 *  @return Placeholder ATM, should return fail or ok
 */
esp_err_t initLSM6DSO (struct i2cDevice dev){

    uint8_t sanityCheckResult;
    readSingleI2cReg(dev, WHO_AM_I, 100, &sanityCheckResult);

    if(sanityCheckResult == 0x6C){
        ESP_LOGI(M_LSM6DSO_TAG, "Sanity check passed.");
    }else{
        ESP_LOGE(M_LSM6DSO_TAG, "Sanity check failed! Got result:, %x", sanityCheckResult);
        return ESP_ERR_INVALID_RESPONSE;
    }

    //set the gyroscope control register to work at 3.33 kHz, 2000 dps and in bypass mode
    writeSingleI2cReg(dev, CTRL2_G, 100, 0b10011000);

    // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
    // low pass filter (check figure9 of LSM6DSOX's datasheet)
    writeSingleI2cReg(dev, CTRL1_XL, 100, 0b10011000);

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    writeSingleI2cReg(dev, CTRL7_G, 100, 0b00000000);

    // Set the ODR config register to ODR/4
    writeSingleI2cReg(dev, CTRL8_XL, 100, 0b00001001);

    // Enable FIFO
    writeSingleI2cReg(dev, EMB_FUNC_EN_B, 100, 0b00001000);

    //
    writeSingleI2cReg(dev, FIFO_CTRL1, 100, 0b11111111);
    writeSingleI2cReg(dev, FIFO_CTRL2, 100, 0b11000001);

    //
    writeSingleI2cReg(dev, FIFO_CTRL3, 100, 0b10101010);
    writeSingleI2cReg(dev, FIFO_CTRL4, 100, 0b00000001);

    return ESP_OK;
}

/*!
 * Reads pressure, in millibars
 * @param dev device that is bound to the sensor
 * @return the measured pressure
 */
esp_err_t getLinearAccelerations(struct i2cDevice dev, struct triplet* tri)
{
    return ESP_OK;
}

/*!
 * Reads pressure, in millibars
 * @param dev device that is bound to the sensor
 * @return the measured pressure
 */
esp_err_t getRotationalAccelerationsLSM6DSO(struct i2cDevice dev, struct triplet* tri)
{
    return ESP_OK;
}

uint16_t getFIFOSampleSize(struct i2cDevice dev )
{
    uint8_t data1, data2;
    readSingleI2cReg(dev, FIFO_STATUS1, 100 / portTICK_PERIOD_MS, &data1);
    readSingleI2cReg(dev, FIFO_STATUS2, 100 / portTICK_PERIOD_MS, &data2);

    uint16_t fifoSize;
    return ((data2 & 0b00000011) << 8) | data1;
}

uint32_t getFIFOSample(struct i2cDevice dev)
{
    /*uint16_t data;
    readDoubleI2cReg(dev, FIFO_DATA_OUT_X_H, 100, &data);
    ESP_LOGI(M_LSM6DSO_TAG, "FIFO read data: %hu", data);*/
    if(checkDataReadyLSM6DSO(dev))
    {
        uint8_t tag;
        readSingleI2cReg(dev, FIFO_DATA_OUT_TAG, 100, &tag);


    } else
    {
        ESP_LOGE(M_LSM6DSO_TAG, "Waiting for FIFO to fill up!");
    }

    return 0;
}

/*!
 * Checks the data ready register. Currently checks if all data is ready to send (i.e. temp, pressure etc)
 * @param dev device that is bound to the sensor
 * @return true if data ready to be read, false otherwise
 */
bool checkDataReadyLSM6DSO(struct i2cDevice dev)
{
    uint8_t dr = 0;
    readSingleI2cReg(dev, FIFO_STATUS1, 100 / portTICK_PERIOD_MS, &dr);
    ESP_LOGD(M_LSM6DSO_TAG, "DR: %x", dr);
    return (dr >> 7);
}