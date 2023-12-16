#include <stdio.h>
#include "MPL3115.h"

/*!
 * MPL3115 specific registers
 */
enum registers {
    STATUS_REGISTER,
    OUT_P_MSB,
    OUT_P_CSB,
    OUT_P_LSB,
    OUT_T_MSB,
    OUT_T_LSB,
    DR_STATUS,
    OUT_P_DELTA_MSB,
    OUT_P_DELTA_CSB,
    OUT_P_DELTA_LSB,
    OUT_T_DELTA_MSB,
    OUT_T_DELTA_LSB,
    WHO_AM_I,
    F_STATUS,
    F_DATA,
    F_SETUP,
    TIME_DLY,
    SYSMOD,
    INT_SOURCE,
    PT_DATA_CFG,
    BAR_IN_MSB,
    BAR_IN_LSB,
    P_TGT_MSB,
    P_TGT_LSB,
    T_TGT,
    P_WND_MSB,
    P_WND_LSB,
    T_WND,
    P_MIN_MSB,
    P_MIN_CSB,
    P_MIN_LSB,
    T_MIN_MSB,
    T_MIN_LSB,
    P_MAX_MSB,
    P_MAX_CSB,
    P_MAX_LSB,
    T_MAX_MSB,
    T_MAX_LSB,
    CTRL_REG1,
    CTRL_REG2,
    CTRL_REG3,
    CTRL_REG4,
    CTRL_REG5,
    OFF_P,
    OFF_T,
    OFF_H

};
/*!
 *  @brief Initializes the sensor
 *  @param dev device to which the barometer binds
 *  @return Placeholder ATM, should return fail or ok
 */
esp_err_t initMPL3115 (struct i2cDevice dev){
    uint8_t sanityCheckResult;
    readSingleI2cReg(dev, 0x0C, 100, &sanityCheckResult);

    if(sanityCheckResult == 0xC4){
            ESP_LOGI(M_MPL3115_TAG, "Sanity check passed.");
    }else{
        ESP_LOGE(M_MPL3115_TAG, "Sanity check failed! Got result:, %x", sanityCheckResult);
        return ESP_ERR_INVALID_RESPONSE;
    }


    //printf("Barometer reports: %x\n", );
    writeSingleI2cReg(dev, CTRL_REG1, 100 / portTICK_PERIOD_MS, 0x0);         // clear config register and halt device
    writeSingleI2cReg(dev, PT_DATA_CFG, 100 / portTICK_PERIOD_MS, 0x07);      // generate deata ready event flags
    writeSingleI2cReg(dev, CTRL_REG1, 100 / portTICK_PERIOD_MS, 0x1);         // start the device

    readSingleI2cReg(dev, STATUS_REGISTER, 100, &sanityCheckResult);
    ESP_LOGI(M_MPL3115_TAG, "Status register reads %x", sanityCheckResult);
    readSingleI2cReg(dev, SYSMOD, 100 / portTICK_PERIOD_MS, &sanityCheckResult);
    ESP_LOGI(M_MPL3115_TAG, "Sysmod register reads %x", sanityCheckResult);

    return ESP_OK;
}

/*!
 * Reads pressure, in millibars
 * @param dev device that is bound to the sensor
 * @return the measured pressure
 */
float getPressureBarMPL3115(struct i2cDevice dev)                    // fixed point, 1000 factor
{
    uint16_t integerPart;
    uint8_t fractionalPart;
    while(!checkDataReadyMPL3115(dev)){
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }  // block until data is available

    readDoubleI2cReg(dev, OUT_P_MSB, 100 / portTICK_PERIOD_MS, &integerPart);
    readSingleI2cReg(dev, OUT_P_LSB, 100 / portTICK_PERIOD_MS, &fractionalPart);

    return (float) ((uint32_t)integerPart << 8 | fractionalPart) /  6400.0f;
}

/*!
 * Checks the data ready register. Currently checks if all data is ready to send (i.e. temp, pressure etc)
 * @param dev device that is bound to the sensor
 * @return true if data ready to be read, false otherwise
 */
bool checkDataReadyMPL3115(struct i2cDevice dev)
{
    uint8_t dr = 0;
    readSingleI2cReg(dev, DR_STATUS, 100 / portTICK_PERIOD_MS, &dr);
    ESP_LOGD(M_MPL3115_TAG, "DR: %x", dr);
    if(dr == 0xee)
        return true;
    else
        return false;
}

/*!
 * Reads temperature, in °C
 * @param dev device that is bound to the sensor
 * @return the temperature measured
 */
float getTemperatureCMPL3115(struct i2cDevice dev){
    while(!checkDataReadyMPL3115(dev)){
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }  // block until data is available
    uint16_t temp;
    readDoubleI2cReg(dev, OUT_T_MSB, 100, &temp);
    return (float)(temp >> 4) / 256.0f;
}