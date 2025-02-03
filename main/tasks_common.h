#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

// WiFi application task
#define WIFI_APP_TASK_STACK_SIZE			4096
#define WIFI_APP_TASK_PRIORITY				5
#define WIFI_APP_TASK_CORE_ID				0

// HTTP Server task
#define HTTP_SERVER_TASK_STACK_SIZE			8192
#define HTTP_SERVER_TASK_PRIORITY			4
#define HTTP_SERVER_TASK_CORE_ID			0

// HTTP Server Monitor task
#define HTTP_SERVER_MONITOR_STACK_SIZE		4096
#define HTTP_SERVER_MONITOR_PRIORITY		3
#define HTTP_SERVER_MONITOR_CORE_ID			0

// IMU task
#define IMU_TASK_STACK_SIZE			        2048
#define IMU_TASK_PRIORITY			        2
#define IMU_TASK_CORE_ID			        0

// Battery task
#define BATTERY_TASK_STACK_SIZE			    2048
#define BATTERY_TASK_PRIORITY			    1
#define BATTERY_TASK_CORE_ID			    0

// Motor task
#define Motor_TASK_STACK_SIZE			    4096
#define Motor_TASK_PRIORITY			        6
#define Motor_TASK_CORE_ID			        0

#endif /* MAIN_TASKS_COMMON_H_ */
