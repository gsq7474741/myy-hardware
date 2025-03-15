#include "mqtt_cb.h"

/* 日志回调函数, SDK的日志会从这里输出 */
int32_t demo_state_logcb(int32_t code, char *message)
{
	printf("%s", message);
	return 0;
}

/* MQTT事件回调函数, 当网络连接/重连/断开时被触发, 事件定义见core/aiot_mqtt_api.h */
void demo_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata)
{
	switch (event->type) {
		/* SDK因为用户调用了aiot_mqtt_connect()接口, 与mqtt服务器建立连接已成功 */
		case AIOT_MQTTEVT_CONNECT: {
			printf("AIOT_MQTTEVT_CONNECT\n");
			/* TODO: 处理SDK建连成功, 不可以在这里调用耗时较长的阻塞函数 */
		} break;

			/* SDK因为网络状况被动断连后, 自动发起重连已成功 */
		case AIOT_MQTTEVT_RECONNECT: {
			printf("AIOT_MQTTEVT_RECONNECT\n");
			/* TODO: 处理SDK重连成功, 不可以在这里调用耗时较长的阻塞函数 */
		} break;

			/* SDK因为网络的状况而被动断开了连接, network是底层读写失败, heartbeat是没有按预期得到服务端心跳应答 */
		case AIOT_MQTTEVT_DISCONNECT: {
			char *cause =
					(event->data.disconnect == AIOT_MQTTDISCONNEVT_NETWORK_DISCONNECT) ? ("network disconnect") : ("heartbeat disconnect");
			printf("AIOT_MQTTEVT_DISCONNECT: %s\n", cause);
			/* TODO: 处理SDK被动断连, 不可以在这里调用耗时较长的阻塞函数 */
		} break;

		default: {
		}
	}
}

/* MQTT默认消息处理回调, 当SDK从服务器收到MQTT消息时, 且无对应用户回调处理时被调用 */
void demo_mqtt_default_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata)
{
	switch (packet->type) {
		case AIOT_MQTTRECV_HEARTBEAT_RESPONSE: {
			printf("heartbeat response\n");
			/* TODO: 处理服务器对心跳的回应, 一般不处理 */
		} break;

		case AIOT_MQTTRECV_SUB_ACK: {
			printf("suback, res: -0x%04lX, packet id: %d, max qos: %d\n",
				   -packet->data.sub_ack.res,
				   packet->data.sub_ack.packet_id,
				   packet->data.sub_ack.max_qos);
			/* TODO: 处理服务器对订阅请求的回应, 一般不处理 */
		} break;

		case AIOT_MQTTRECV_PUB: {
			printf("pub, qos: %d, topic: %.*s\n", packet->data.pub.qos, packet->data.pub.topic_len, packet->data.pub.topic);
			// printf("pub, payload: %.*s\n", packet->data.pub.payload_len, packet->data.pub.payload);
			printf("pub, payload: %.*s\n", (int) packet->data.pub.payload_len, packet->data.pub.payload);
			/* TODO: 处理服务器下发的业务报文 */
		} break;

		case AIOT_MQTTRECV_PUB_ACK: {
			printf("puback, packet id: %d\n", packet->data.pub_ack.packet_id);
			/* TODO: 处理服务器对QoS1上报消息的回应, 一般不处理 */
		} break;

		default: {
		}
	}
}
