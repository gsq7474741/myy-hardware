#ifndef __SEND_H__
#define __SEND_H__

#include "main.h"

/* 属性上报函数演示 */
// 发送单个属性数据
int32_t demo_send_property_post(void *dm_handle, char *params);

// 发送多个属性数据
int32_t demo_send_property_batch_post(void *dm_handle, char *params);

/* 事件上报函数演示 */
// 例如开/关等事件
int32_t demo_send_event_post(void *dm_handle, char *event_id, char *params);

#endif
