#include "motor.h"

static const char *TAG = "Motor";

struct Motor motor;

static TaskHandle_t task_motor= NULL;

void motor_init(void){
    //PWM Timer init
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_timer_config_t ledc_timer_1 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_1));

    ledc_timer_config_t ledc_timer_2 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_2,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_2));

    ledc_timer_config_t ledc_timer_3 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_3,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_3));

    //PWM Channel init
    ledc_channel_config_t ledc_channel[6];
    ledc_channel[0].channel = LEDC_CHANNEL_M1_L;
    ledc_channel[0].gpio_num = MOTOR_1A;
    ledc_channel[0].timer_sel = LEDC_TIMER_0;
    ledc_channel[1].channel = LEDC_CHANNEL_M1_H;
    ledc_channel[1].gpio_num = MOTOR_1B;
    ledc_channel[1].timer_sel = LEDC_TIMER_0;

    ledc_channel[2].channel = LEDC_CHANNEL_M2_L;
    ledc_channel[2].gpio_num = MOTOR_2A;
    ledc_channel[2].timer_sel = LEDC_TIMER_0;
    ledc_channel[3].channel = LEDC_CHANNEL_M2_H;
    ledc_channel[3].gpio_num = MOTOR_2B;
    ledc_channel[3].timer_sel = LEDC_TIMER_0;

    ledc_channel[4].channel = LEDC_CHANNEL_M3_L;
    ledc_channel[4].gpio_num = MOTOR_3A;
    ledc_channel[4].timer_sel = LEDC_TIMER_0;
    ledc_channel[5].channel = LEDC_CHANNEL_M3_H;
    ledc_channel[5].gpio_num = MOTOR_3B;
    ledc_channel[5].timer_sel = LEDC_TIMER_0;

    for (int i = 0; i < 6; i++)
    {   
        ledc_channel[i].speed_mode = LEDC_MODE;
        ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
        ledc_channel[i].duty = 0;
        ledc_channel[i].hpoint = 0;

        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
        ESP_LOGI(TAG," PWM channel %d is initiated.", i);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void move_motor(ledc_channel_t channel_1, ledc_channel_t channel_2, int16_t speed){
    int16_t speed_abs = abs(speed);

    if(speed > 0){
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_1, speed_abs));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_1));
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_2, 0));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_2));
    }
    else if(speed < 0){
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_1, 0));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_1));
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_2, speed_abs));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_2));
    }
    else{
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_1, 0));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_1));
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel_2, 0));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel_2));
    }
}

void get_val(char joystick){
    motor.linear_acc = 500;
    motor.rot_acc = 100;

    switch (joystick){
        case '0':
            // ESP_LOGI(TAG, "Robot Stops");
            motor.loc_x = 0;
            motor.loc_y = 0;
            motor.ang_z = 0;
            break;
        case '1':
            // ESP_LOGI(TAG, "Moving Forward");
            motor.loc_x = 0;
            motor.loc_y += motor.linear_acc;
            motor.ang_z = 0;
            break;
        case '2':
            // ESP_LOGI(TAG, "Moving Backward");
            motor.loc_x = 0;
            motor.loc_y -= motor.linear_acc;
            motor.ang_z = 0;
            break;
        case '3':
            // ESP_LOGI(TAG, "Moving Left");
            motor.loc_x -= motor.linear_acc;
            motor.loc_y = 0;
            motor.ang_z = 0;
            break;
        case '4':
            // ESP_LOGI(TAG, "Moving Right");
            motor.loc_x += motor.linear_acc;
            motor.loc_y = 0;
            motor.ang_z = 0;
            break;
        case '5':
            // ESP_LOGI(TAG, "Left Forward");
            motor.loc_x -= motor.linear_acc;
            motor.loc_y += motor.linear_acc;
            motor.ang_z = 0;
            break;
        case '6':
            // ESP_LOGI(TAG, "Right Forward");
            motor.loc_x += motor.linear_acc;
            motor.loc_y += motor.linear_acc;
            motor.ang_z = 0;
            break;
        case '7':
            // ESP_LOGI(TAG, "Left Backward");
            motor.loc_x -= motor.linear_acc;
            motor.loc_y -= motor.linear_acc;
            motor.ang_z = 0;
            break;
        case '8':
            // ESP_LOGI(TAG, "Right Backward");
            motor.loc_x += motor.linear_acc;
            motor.loc_y -= motor.linear_acc;
            motor.ang_z = 0;
            break;
        case 'L':
            // ESP_LOGI(TAG, "Counter Clockwise");
            motor.loc_x = 0;
            motor.loc_y = 0;
            motor.ang_z -= motor.rot_acc;
            break;
        case 'R':
            // ESP_LOGI(TAG, "Clockwise");
            motor.loc_x = 0;
            motor.loc_y = 0;
            motor.ang_z += motor.rot_acc;
            break;
        
    }
}

void get_kinematics(){
    motor.m1_val = ((1 * motor.loc_x)      + (0 * motor.loc_y)              + (Length * motor.ang_z))/Radius;
    motor.m2_val = (((-1/2) * motor.loc_x) + ((-sin(M_PI/3)) * motor.loc_y) + (-Length * motor.ang_z))/Radius;
    motor.m3_val = (((-1/2) * motor.loc_x) + ((sin(M_PI/3)) * motor.loc_y)  + (-Length * motor.ang_z))/Radius;

    // pwm speed limit
    if(motor.m1_val >= 255){
        motor.m1_val = 255;
    }
    else if (motor.m1_val <-255){
        motor.m1_val = -255;
    }
    if(motor.m2_val >= 255){
        motor.m2_val = 255;
    }
    else if (motor.m2_val <-255){
        motor.m2_val = -255;
    }
    if(motor.m3_val >= 255){
        motor.m3_val = 255;
    }
    else if (motor.m3_val <-255){
        motor.m3_val = -255;
    }
}

static void motor_task(void *pvParameters){
    gpio_reset_pin(nSleep_pin);
    ESP_ERROR_CHECK(gpio_set_direction(nSleep_pin ,GPIO_MODE_OUTPUT));
    motor_init();
    // motor.speed = 255;
    while(1){
        // ESP_LOGI(TAG, "Controller is %c", motor.controller);
        get_val(motor.controller);
        get_kinematics();
        // ESP_LOGI(TAG, "M1 = %d , M2 = %d , M3 = %d", motor.m1_val, motor.m2_val, motor.m3_val);
        gpio_set_level(nSleep_pin, 1);
        move_motor(LEDC_CHANNEL_M1_L, LEDC_CHANNEL_M1_H, motor.m1_val);
        move_motor(LEDC_CHANNEL_M2_L, LEDC_CHANNEL_M2_H, motor.m2_val);
        move_motor(LEDC_CHANNEL_M3_L, LEDC_CHANNEL_M3_H, motor.m3_val);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void motor_start(void){
    xTaskCreatePinnedToCore(&motor_task, "Motor Initiate", Motor_TASK_STACK_SIZE, NULL, Motor_TASK_PRIORITY, &task_motor, Motor_TASK_CORE_ID);
}
