#include "battery.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks_common.h"
#include "driver/adc.h"
#include "esp_log.h"

static TaskHandle_t task_battery= NULL;

static const char *TAG = "Battery";

struct Battery batt;

float mapfloat(float val, float in_min, float in_max, float out_min, float out_max);

static void battery_task(void *pvParameters){
    // CE pin
    gpio_set_direction(CE_pin ,GPIO_MODE_OUTPUT);

    // ADC channel
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);

    while(1){
        // High battery charger IC
        gpio_set_level(CE_pin, 1);

        // ADC for battery voltage and percent monitor
        float voltage_level = adc1_get_raw(ADC1_CHANNEL_2);
        // Get the voltage level with unit "V".
        batt.voltage_level = (voltage_level * (RefVoltage / 4095.00) * 2) + Calibration;
        ESP_LOGI(TAG," Voltage level is %.2f .", batt.voltage_level);
        // Get the voltage level with unit "%".
        batt.volts_perc = mapfloat(batt.voltage_level, RefVoltage, MaxVoltage, 0, 100);
        ESP_LOGI(TAG," Battery percent is %.2f.", batt.volts_perc);
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void battery_start(void){
    xTaskCreatePinnedToCore(&battery_task, "Battery Initiate", BATTERY_TASK_STACK_SIZE, NULL, BATTERY_TASK_PRIORITY, &task_battery, BATTERY_TASK_CORE_ID);
}

float mapfloat(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}