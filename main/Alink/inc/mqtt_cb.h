#ifndef _MQTT_HANDLE_H_
#define _MQTT_HANDLE_H_

#include "main.h"

/* 定义日志回调函数 */
int32_t demo_state_logcb(int32_t code, char *message);

/* MQTT事件回调函数 */
void demo_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata);

/* MQTT默认消息处理回调 */
void demo_mqtt_default_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata);

#endif





