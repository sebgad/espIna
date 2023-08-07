#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "espIna.h"

extern "C" void app_main()
{

    espIna objIna;

    // Setup devices
    ESP_ERROR_CHECK(objIna.set_calibration(0.1F, 3.2F));
    float bus_voltage = 0.F;
    float bus_current = 0.F;
    ESP_ERROR_CHECK(objIna.adjust_current_offset(0.2F));

    while(1){
        objIna.get_bus_voltage(&bus_voltage);
        objIna.get_bus_current(&bus_current);

        ESP_LOGI("INA219", "Bus Voltage is %.4f V", bus_voltage);
        ESP_LOGI("INA219", "Bus Current is %.4f A", bus_current);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
