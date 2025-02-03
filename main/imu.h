#ifndef _IMU_H_
#define _IMU_H_

#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           7           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           6           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// Sensor Address
#define ACCEL_SENSOR_ADDR           0x18
#define GYRO_SENSOR_ADDR            0x68

// Accelerometer registers
#define ACCEL_CHIP_DATA             0x00
#define ACCEL_CONF                  0x40
#define ACCEL_RANGE                 0x41
#define ACCEL_SELFTEST              0x6D
#define ACCEL_PWR_CONF              0x7C
#define ACCEL_PWR_CNTRL             0x7D
#define ACCEL_SOFTRESET             0x7E

// Accelerometer param
#define OSR4                        0x08
#define OSR2                        0x09
#define Normal_OSR                  0x0A

#define ODR_12_5                    0x05
#define ODR_25                      0x06
#define ODR_50                      0x07
#define ODR_100                     0x08
#define ODR_200                     0x09
#define ODR_400                     0x0A
#define ODR_800                     0x0B
#define ODR_1600                    0x0C

// Gyroscope registers
#define GYRO_CHIP_DATA              0x00
#define GYRO_RANGE                  0x0F
#define GYRO_BW                     0x10
#define GYRO_PM                     0x11
#define GYRO_SOFTRESET              0X14
#define GYRO_SELFTEST               0x3C

// Sensor Data
#define ACCEL_X_LSB                 0x12
#define SENSOR_TIME                 0x18
#define TEMP_OUT_H                  0x22
#define GYRO_X_LSB                  0x02

// Structures
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
};

struct SensorData
{
    int16_t ax, ay, az, gx, gy, gz;
};

struct GyroCal
{
    float x, y, z;
};

struct Attitude
{
    float r, p, y;
};

extern struct RawData rawData;
extern struct SensorData sensorData;
extern struct GyroCal gyroCal;
extern struct Attitude attitude;

void imu_Init(void);
void self_test(void);
void readAccel(void);
void readGyro(void);
void calibrateGyro(uint16_t numCalPoints);
void readTemp(void);
void readRawData(void);
void readProcessedData(void);
void calcAtt_Accel(void);
void calcAtt_Gyro(void);
void calcAtt_Comp(void);
void calcAtt_EKF(void);
void imu_start(void);

#endif /* _IMU_H_ */