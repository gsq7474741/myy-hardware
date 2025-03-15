#include "report.h"

// 处理服务器下发的属性设置，比如温度设置
void demo_dm_recv_property_set(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_property_set msg_id = %ld, params = %.*s\r\n",
		   (unsigned long) recv->data.property_set.msg_id,
		   (int) recv->data.property_set.params_len,
		   recv->data.property_set.params);
	// 设备云端在线调试控制电磁阀

	// 临时添加终止符
	char *temp_params = malloc(recv->data.property_set.params_len + 1);
	if (temp_params == NULL) {
		printf("Memory allocation failed\n");
		return;
	}
	memcpy(temp_params, recv->data.property_set.params, recv->data.property_set.params_len);
	temp_params[recv->data.property_set.params_len] = '\0';

	if (strcmp(temp_params, "{\"WaterOutletSwitch\":0}") == 0) {
		// Solenoid_valves_close();
		water_switch = false;
		solenoid_valves_close();
		// WaterOutletSwitch_send_property_post(dm_handle,0);
	}
	if (strcmp(temp_params, "{\"WaterOutletSwitch\":1}") == 0) {
		// Solenoid_valves_open();
		water_switch = true;
		solenoid_valves_open();
		// WaterOutletSwitch_send_property_post(dm_handle,1);
	}
}

// 这个函数用于处理服务器发送给客户端的回复消息。当客户端向服务器发送请求（例如，查询设备状态、更新配置等）
// 服务器会对这些请求做出响应，回复消息会包含请求的结果和其他相关信息。
void demo_dm_recv_generic_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_generic_reply msg_id = %lu, code = %lu, data = %.*s, message = %.*s\r\n",
		   recv->data.generic_reply.msg_id,
		   recv->data.generic_reply.code,
		   (int) recv->data.generic_reply.data_len,
		   recv->data.generic_reply.data,
		   (int) recv->data.generic_reply.message_len,
		   recv->data.generic_reply.message);
}

// 可设置设备的开关状态，用于处理设备管理（Device Management，DM）接收到的异步服务调用请求。
void demo_dm_recv_async_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_async_service_invoke msg_id = %ld, service_id = %s, params = %.*s\r\n",
		   (unsigned long) recv->data.async_service_invoke.msg_id,
		   recv->data.async_service_invoke.service_id,
		   (int) recv->data.async_service_invoke.params_len,
		   recv->data.async_service_invoke.params);
}

void demo_dm_recv_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_sync_service_invoke msg_id = %ld, rrpc_id = %s, service_id = %s, params = %.*s\r\n",
		   (unsigned long) recv->data.sync_service_invoke.msg_id,
		   recv->data.sync_service_invoke.rrpc_id,
		   recv->data.sync_service_invoke.service_id,
		   (int) recv->data.sync_service_invoke.params_len,
		   recv->data.sync_service_invoke.params);
}

void demo_dm_recv_raw_data(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_raw_data raw data len = %lu\r\n", recv->data.raw_data.data_len);
}

// 同步服务调用请求（raw sync service invoke）是一种在客户端和服务器之间进行数据交互的方式，特别是在物联网（IoT）场景中，
// 这种调用方式允许客户端发送请求并在收到服务器响应之前阻塞等待。
// 这种方式保证了请求的顺序执行，并且在处理结果之前不会继续执行后续代码。
void demo_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_raw_sync_service_invoke raw sync service rrpc_id = %s, data_len = %lu\r\n",
		   recv->data.raw_service_invoke.rrpc_id,
		   recv->data.raw_service_invoke.data_len);
}

void demo_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_raw_data_reply receive reply for up_raw msg, data len = %lu\r\n", recv->data.raw_data.data_len);
	/* TODO: 用户处理下行的二进制数据, 位于recv->data.raw_data.data中 */
}

/* 用户数据接收处理回调函数 */
void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
	printf("demo_dm_recv_handler, type = %d\r\n", recv->type);

	switch (recv->type) {

		/* 属性上报, 事件上报, 获取期望属性值或者删除期望属性值的应答 */
		case AIOT_DMRECV_GENERIC_REPLY: {
			demo_dm_recv_generic_reply(dm_handle, recv, userdata);
		} break;

			/* 属性设置 */
		case AIOT_DMRECV_PROPERTY_SET: {
			demo_dm_recv_property_set(dm_handle, recv, userdata);
		} break;

			/* 异步服务调用 */
		case AIOT_DMRECV_ASYNC_SERVICE_INVOKE: {
			demo_dm_recv_async_service_invoke(dm_handle, recv, userdata);
		} break;

			/* 同步服务调用 */
		case AIOT_DMRECV_SYNC_SERVICE_INVOKE: {
			demo_dm_recv_sync_service_invoke(dm_handle, recv, userdata);
		} break;

			/* 下行二进制数据 */
		case AIOT_DMRECV_RAW_DATA: {
			demo_dm_recv_raw_data(dm_handle, recv, userdata);
		} break;

			/* 二进制格式的同步服务调用, 比单纯的二进制数据消息多了个rrpc_id */
		case AIOT_DMRECV_RAW_SYNC_SERVICE_INVOKE: {
			demo_dm_recv_raw_sync_service_invoke(dm_handle, recv, userdata);
		} break;

			/* 上行二进制数据后, 云端的回复报文 */
		case AIOT_DMRECV_RAW_DATA_REPLY: {
			demo_dm_recv_raw_data_reply(dm_handle, recv, userdata);
		} break;

		default:
			break;
	}
}
