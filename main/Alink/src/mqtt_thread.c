#include "mqtt_thread.h"

/* 执行aiot_mqtt_process的线程, 包含心跳发送和QoS1消息重发 */
void *demo_mqtt_process_thread(void *args)
{
	int32_t res = STATE_SUCCESS;

	while (g_mqtt_process_thread_running) {
		res = aiot_mqtt_process(args);
		if (res == STATE_USER_INPUT_EXEC_DISABLED) {
			break;
		}
		sleep(1);
	}
	return NULL;
}

/* 执行aiot_mqtt_recv的线程, 包含网络自动重连和从服务器收取MQTT消息 */
void *demo_mqtt_recv_thread(void *args)
{
	int32_t res = STATE_SUCCESS;

	while (g_mqtt_recv_thread_running) {
		res = aiot_mqtt_recv(args);
		if (res < STATE_SUCCESS) {
			if (res == STATE_USER_INPUT_EXEC_DISABLED) {
				break;
			}
			sleep(1);
		}
	}
	return NULL;
}
/******************************************************************************************************/
