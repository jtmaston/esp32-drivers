#include <sys/queue.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include "i2cDriverCommon.h"
#include "LM75.h"
#include "MPL3115.h"
#include <esp_event.h>

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
        ESP_LOGI("MPL3115", "Pressure read: %f", getPressureBarMPL3115(barometer));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initBus(2, 1, I2C_NUM_0, 400000, false);


    xTaskCreate(testMPL3115, "testMPL3115", 2048, NULL,
                5, NULL);

    xTaskCreate(testLM75, "testLM75", 2048, NULL,
                5, NULL);

}
