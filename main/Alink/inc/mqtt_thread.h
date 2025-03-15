#ifndef _MQTT_THREAD_H_
#define _MQTT_THREAD_H_

#include "main.h"

extern uint8_t g_mqtt_process_thread_running;
extern uint8_t g_mqtt_recv_thread_running;
extern uint8_t ota_running;

/* 执行aiot_mqtt_process的线程, 包含心跳发送和QoS1消息重发 */
void *demo_mqtt_process_thread(void *args);

/* 执行aiot_mqtt_recv的线程, 包含网络自动重连和从服务器收取MQTT消息 */
void *demo_mqtt_recv_thread(void *args);

#endif





