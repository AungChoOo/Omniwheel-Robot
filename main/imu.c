#include "imu.h"
#include "tasks_common.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

struct RawData rawData;
struct SensorData sensorData;
struct GyroCal gyroCal;
struct Attitude attitude;

static const char *TAG = "IMU";

static TaskHandle_t task_imu= NULL;

static esp_err_t i2c_master_init(void){
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
};

static esp_err_t register_read(uint8_t sen_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, sen_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t register_write_byte(uint8_t sen_addr, uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, sen_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

void imu_Init(void){
    ESP_ERROR_CHECK(i2c_master_init());
    // ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_SOFTRESET, 0xB6));
    // ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_SOFTRESET, 0xB6));
    
    // Power Mode ON
    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_PWR_CONF, 0));
    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_PWR_CNTRL, 4));
    ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_PM, 0));
    
    // Get the IDs
    uint8_t sen_id[1];
    ESP_ERROR_CHECK(register_read(ACCEL_SENSOR_ADDR, ACCEL_CHIP_DATA, sen_id, 1));
    ESP_LOGI(TAG,"Accelerometer's address is %d", sen_id[0]);
    ESP_ERROR_CHECK(register_read(GYRO_SENSOR_ADDR, GYRO_CHIP_DATA, sen_id, 1));
    ESP_LOGI(TAG,"Gyroscope's address is %d", sen_id[0]);

    // Set the ODR and Range
    uint8_t ODR[1];
    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_CONF, (0x0A << 4) | 0x0C));
    ESP_ERROR_CHECK(register_read(ACCEL_SENSOR_ADDR, ACCEL_CONF, ODR, 1));
    ESP_LOGI(TAG,"Accelerometer's conf is %hhu",ODR[0]);
    ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_BW, (0x01)));
    ESP_ERROR_CHECK(register_read(GYRO_SENSOR_ADDR, GYRO_BW, ODR, 1));
    ESP_LOGI(TAG,"Gyroscope's conf is %hhu",ODR[0]);

    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_RANGE, 0x00));
    ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_RANGE, 0x00));
}

void self_test(void){
    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_SELFTEST, 0x0D));
    ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_SELFTEST, 0x01));

    uint8_t test[1];
    ESP_ERROR_CHECK(register_read(ACCEL_SENSOR_ADDR, ACCEL_SELFTEST, test, 1));
    ESP_LOGI(TAG,"Accelerometer's test result is %hhu",test[0]);
    ESP_ERROR_CHECK(register_read(GYRO_SENSOR_ADDR, GYRO_SELFTEST, test, 1));
    ESP_LOGI(TAG,"Gyroscope's test result is %hhu",test[0]);

    ESP_ERROR_CHECK(register_write_byte(ACCEL_SENSOR_ADDR, ACCEL_SELFTEST, 0x00));
    ESP_ERROR_CHECK(register_write_byte(GYRO_SENSOR_ADDR, GYRO_SELFTEST, 0x00));
}

void readAccel(void){
    uint8_t accel_raw[6];
    ESP_ERROR_CHECK(register_read(ACCEL_SENSOR_ADDR, ACCEL_X_LSB, accel_raw, 6));
    rawData.ax = (accel_raw[1] << 8) | accel_raw[0];
    rawData.ay = (accel_raw[3] << 8) | accel_raw[2];
    rawData.az = (accel_raw[5] << 8) | accel_raw[4];
}

void readTemp(void){
    uint8_t temp_raw[2];
    ESP_ERROR_CHECK(register_read(ACCEL_SENSOR_ADDR, TEMP_OUT_H, temp_raw, 14));
    rawData.t = (temp_raw[0] << 8 | temp_raw[1]);
}

void readGyro(void){
    uint8_t gyro_raw[6];
    ESP_ERROR_CHECK(register_read(GYRO_SENSOR_ADDR, GYRO_X_LSB, gyro_raw, 6));
    rawData.gx = (gyro_raw[1] << 8) | gyro_raw[0];
    rawData.gy = (gyro_raw[3] << 8) | gyro_raw[2];
    rawData.gz = (gyro_raw[5] << 8) | gyro_raw[4];
}

void calibrateGyro(uint16_t numCalPoints){

    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        readGyro();
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        printf("Gyro is caliberating : %d \n", ii);
        
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
    ESP_LOGI(TAG,"Cal X = %.2f Cal Y = %.2f Cal Z = %.2f ", gyroCal.x, gyroCal.y, gyroCal.z);
}

void readRawData(void){
    readAccel();
    // readTemp();
    readGyro();
}

void readProcessedData(void){
    readRawData();
    // Convert accelerometer values to g's
    sensorData.ax = (rawData.ax * 3 * 9.807) / 32768;
    sensorData.ay = (rawData.ay * 3 * 9.807) / 32768;
    sensorData.az = (rawData.az * 3 * 9.807) / 32768;

    // Convert temperature values to 'C
    // sensorData.tc = (rawData.t/340) + 36.53;

    // Convert gyro values to deg/s
    sensorData.gx = (rawData.gx - gyroCal.x)*(16.384);
    sensorData.gy = (rawData.gy - gyroCal.y)*(16.384);
    sensorData.gz = (rawData.gz - gyroCal.z)*(16.384);
}

static void imu_task(void *pvParameters){
    imu_Init();
    ESP_LOGI(TAG, "IMU initialized successfully");
    self_test();
    ESP_LOGI(TAG, "IMU self testing finished successfully");
    calibrateGyro(1000);
    ESP_LOGI(TAG, "Gyro caliberated successfully");

    while(1){
        readProcessedData();
        vTaskDelay(1000/portTICK_PERIOD_MS); 
    }
}

void imu_start(void){
    xTaskCreatePinnedToCore(&imu_task, "Imu RPY", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &task_imu, IMU_TASK_CORE_ID);
}