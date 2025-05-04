#ifndef __MAIN_H__
#define __MAIN_H__

#include <string.h>  // 包含 bzero
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aiot_dm_api.h"
#include "aiot_mqtt_api.h"
#include "aiot_mqtt_download_api.h"
#include "aiot_ota_api.h"
#include "aiot_state_api.h"
#include "aiot_sysdep_api.h"

/* 订阅发布话题 */
#define MQTT_SUB_TOPIC_PROPERTY "/sys/" CONFIG_PRODUCT_KEY "/" CONFIG_DEVICE_NAME "/thing/event/+/post_reply"
#define MQTT_PUB_TOPIC_PROPERTY "/sys/" CONFIG_PRODUCT_KEY "/" CONFIG_DEVICE_NAME "/thing/event/property/post"

/* 任务调度 */
#define PUB_TIME 4.5  //秒
#define MODBUS_TASK_TIME 2  //秒

/* RS485相关宏定义 */
#define TXD_PIN         17        // RS485 TX pin
#define RXD_PIN         18        // RS485 RX pin
#define RECV_ENABLE_PIN GPIO_NUM_1// RS485 接收使能 pin
#define UART_PORT_NUM   1
#define BUF_SIZE_485        (1024)

/* 外设LED电磁阀引脚定义 */
#define SOLENOID_VALVES_PIN GPIO_NUM_2// 电磁阀 pin
#define SOLENOID_LED_PIN GPIO_NUM_38// 电磁阀指示灯 pin

/* WIFI重连等待时间 */
#define WIFI_WAIT_TIME 1 //秒

void hardware_init(void);
void wifi_start(void);
int alink_main(void);

#endif
