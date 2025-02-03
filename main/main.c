#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "imu.h"
#include "battery.h"
#include "motor.h"

static const char *TAG = "Omni-wheel";

void app_main(void)
{	
    // Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Start Wifi
	wifi_app_start();
	ESP_LOGI(TAG, "WIFI_START");

	// Start battery
	battery_start();
	ESP_LOGI(TAG, "Battery_START");

	// Start IMU
    imu_start();
	ESP_LOGI(TAG, "IMU_START");

	// Start Motor
	motor_start();
	ESP_LOGI(TAG, "Motor_START");
}