#include "send.h"

/* 属性上报函数演示 */
// 发送单个属性数据
int32_t demo_send_property_post(void *dm_handle, char *params)
{
	aiot_dm_msg_t msg;

	memset(&msg, 0, sizeof(aiot_dm_msg_t));
	msg.type                      = AIOT_DMMSG_PROPERTY_POST;
	msg.data.property_post.params = params;

	return aiot_dm_send(dm_handle, &msg);
}

// 发送多个属性数据
int32_t demo_send_property_batch_post(void *dm_handle, char *params)
{
	aiot_dm_msg_t msg;

	memset(&msg, 0, sizeof(aiot_dm_msg_t));
	msg.type                      = AIOT_DMMSG_PROPERTY_BATCH_POST;
	msg.data.property_post.params = params;

	return aiot_dm_send(dm_handle, &msg);
}

/* 事件上报函数演示 */
// 例如开/关等事件
int32_t demo_send_event_post(void *dm_handle, char *event_id, char *params)
{
	aiot_dm_msg_t msg;

	memset(&msg, 0, sizeof(aiot_dm_msg_t));
	msg.type                     = AIOT_DMMSG_EVENT_POST;
	msg.data.event_post.event_id = event_id;
	msg.data.event_post.params   = params;

	return aiot_dm_send(dm_handle, &msg);
}