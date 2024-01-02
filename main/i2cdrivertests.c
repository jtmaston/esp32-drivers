#include <sys/queue.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include "i2cDriverCommon.h"
#include "LM75.h"
#include "MPL3115.h"
#include "LSM6DSO.h"
#include <esp_event.h>
#include <string.h>

_Noreturn static void testLM75(void *pvParameters) {
    struct i2cDevice tempSensor = {I2C_NUM_0, 0x49};
    while(true) {
        float data = readTempLM75(tempSensor, 100) / 100.0f;
        ESP_LOGI("LM75", "Temp read: %f", data);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void testMPL3115(void *pvParameters) {
    struct i2cDevice barometer = {I2C_NUM_0, 0x60};
    initMPL3115(barometer);

    while(true) {
        ESP_LOGI("MPL3115", "Pressure read: %f | Temp read: %f", getPressureBarMPL3115(barometer),
                 getTemperatureCMPL3115(barometer));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void testLSM6DSO(void *pvParameters) {
    struct i2cDevice IMU = {I2C_NUM_0, 0x6A};

    typedef union {
        int16_t i16bit[3];
        uint8_t u8bit[6];
    } axis3bit16_t;

     float acceleration_mg[3];
     float angular_rate_mdps[3];
     uint8_t tx_buffer[1000];

    axis3bit16_t data_raw_acceleration;
    axis3bit16_t data_raw_angular_rate;

    stmdev_ctx_t imuctx;
    imuctx.write_reg = platform_write;
    imuctx.read_reg = platform_read;
    imuctx.handle = (void*) &IMU;

    uint8_t sanityCheckResult;
    lsm6dso_device_id_get(&imuctx, &sanityCheckResult);

    if(sanityCheckResult == 0x6C){
        ESP_LOGI(M_LSM6DSO_TAG, "Sanity check passed.");
    }else{
        ESP_LOGE(M_LSM6DSO_TAG, "Sanity check failed! Got result:, %x", sanityCheckResult);
    }

    lsm6dso_reset_set(&imuctx, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lsm6dso_reset_get(&imuctx, &rst);
    } while (rst);

    lsm6dso_i3c_disable_set(&imuctx, LSM6DSO_I3C_DISABLE);
    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&imuctx, PROPERTY_ENABLE);
    /* Set full scale */
    lsm6dso_xl_full_scale_set(&imuctx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&imuctx, LSM6DSO_125dps);
    /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to 10 samples
     */
    lsm6dso_fifo_watermark_set(&imuctx, 12000);
    /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
    lsm6dso_fifo_xl_batch_set(&imuctx, LSM6DSO_XL_BATCHED_AT_3333Hz);
    lsm6dso_fifo_gy_batch_set(&imuctx, LSM6DSO_GY_BATCHED_AT_417Hz);
    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dso_fifo_mode_set(&imuctx, LSM6DSO_STREAM_MODE);
    /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
    //lsm6dso_data_ready_mode_set(&imuctx, LSM6DSO_DRDY_PULSED);
    /* Uncomment if interrupt generation on Free Fall INT1 pin */
    //lsm6dso_pin_int1_route_get(&imuctx, &int1_route);
    //int1_route.reg.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
    //lsm6dso_pin_int1_route_set(&imuctx, &int1_route);
    /* Uncomment if interrupt generation on Free Fall INT2 pin */
    //lsm6dso_pin_int2_route_get(&imuctx, &int2_route);
    //int2_route.reg.int2_ctrl.int2_fifo_th = PROPERTY_ENABLE;
    //lsm6dso_pin_int2_route_set(&imuctx, &int2_route);
    /* Set Output Data Rate */
    lsm6dso_xl_data_rate_set(&imuctx, LSM6DSO_XL_ODR_3333Hz);
    lsm6dso_gy_data_rate_set(&imuctx, LSM6DSO_GY_ODR_417Hz);

    while (1) {
        uint16_t num = 0;
        uint8_t wmflag = 0;
        lsm6dso_fifo_tag_t reg_tag;
        axis3bit16_t dummy;
        /* Read watermark flag */
        lsm6dso_fifo_wtm_flag_get(&imuctx, &wmflag);

        if (wmflag > 0) {
            /* Read number of samples in FIFO */
            lsm6dso_fifo_data_level_get(&imuctx, &num);

            while (num--) {
                /* Read FIFO tag */
                lsm6dso_fifo_sensor_tag_get(&imuctx, &reg_tag);

                switch (reg_tag) {
                    case LSM6DSO_XL_NC_TAG:
                        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
                        lsm6dso_fifo_out_raw_get(&imuctx, data_raw_acceleration.u8bit);
                        acceleration_mg[0] =
                                lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
                        acceleration_mg[1] =
                                lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
                        acceleration_mg[2] =
                                lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
                                ESP_LOGI("LSM6DSO:", "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f",
                                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
                        break;

                    case LSM6DSO_GYRO_NC_TAG:
                        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
                        lsm6dso_fifo_out_raw_get(&imuctx, data_raw_angular_rate.u8bit);
                        angular_rate_mdps[0] =
                                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
                        angular_rate_mdps[1] =
                                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
                        angular_rate_mdps[2] =
                                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
                        ESP_LOGI("LSM6DSO:", "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f",
                                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
                        break;

                    default:
                        /* Flush unused samples */
                        memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
                        lsm6dso_fifo_out_raw_get(&imuctx, dummy.u8bit);
                        break;
                }
            }
        }
    }
}



void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initBus(2, 1, I2C_NUM_0, 400000, false);

    xTaskCreate(testMPL3115, "testMPL3115", 2048, NULL,
                5, NULL);

    //xTaskCreate(testLM75, "testLM75", 2048, NULL,
    //            5, NULL);

    xTaskCreate(testLSM6DSO, "testLSM6DSO", 2048, NULL,
                5, NULL);

}
