#ifndef _Motor_H_
#define _Motor_H_

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks_common.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES         LEDC_TIMER_8_BIT     // Set duty resolution to 13 bits
#define LEDC_FREQUENCY        5000                 // Frequency in Hertz. Set frequency at 5 kHz

#define nSleep_pin             20
#define MOTOR_1A               18
#define MOTOR_1B               19
#define MOTOR_2A               22
#define MOTOR_2B               21
#define MOTOR_3A               11
#define MOTOR_3B               10

#define LEDC_CHANNEL_M1_L       (LEDC_CHANNEL_0)
#define LEDC_CHANNEL_M1_H       (LEDC_CHANNEL_1)
#define LEDC_CHANNEL_M2_L       (LEDC_CHANNEL_2)
#define LEDC_CHANNEL_M2_H       (LEDC_CHANNEL_3)
#define LEDC_CHANNEL_M3_L       (LEDC_CHANNEL_4)
#define LEDC_CHANNEL_M3_H       (LEDC_CHANNEL_5)

#define Length                  48
#define Radius                  28.5

struct Motor{
    int16_t speed;
    uint8_t controller;
    int linear_acc, rot_acc;
    int loc_x, loc_y, ang_z;
    int m1_val, m2_val, m3_val;
};

extern struct Motor motor;

void motor_init(void);
void move_motor(ledc_channel_t channel_1, ledc_channel_t channel_2, int16_t speed);
void get_val(char joystick);
void get_kinematics();
void motor_start(void);

#endif /* _Motor_H_ */
