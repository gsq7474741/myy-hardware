#ifndef __REPORT_H__
#define __REPORT_H__

#include "main.h"
#include "solenoid.h"

extern bool water_switch;

// 这个函数用于处理服务器发送给客户端的回复消息。当客户端向服务器发送请求（例如，查询设备状态、更新配置等）
// 服务器会对这些请求做出响应，回复消息会包含请求的结果和其他相关信息。
void demo_dm_recv_generic_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

// 处理服务器下发的属性设置，比如温度设置
void demo_dm_recv_property_set(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

// 可设置设备的开关状态，用于处理设备管理（Device Management，DM）接收到的异步服务调用请求。
void demo_dm_recv_async_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

void demo_dm_recv_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

void demo_dm_recv_raw_data(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

// 同步服务调用请求（raw sync service invoke）是一种在客户端和服务器之间进行数据交互的方式，特别是在物联网（IoT）场景中，
// 这种调用方式允许客户端发送请求并在收到服务器响应之前阻塞等待。
// 这种方式保证了请求的顺序执行，并且在处理结果之前不会继续执行后续代码。
void demo_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

void demo_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

/* 用户数据接收处理回调函数 */
void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

#endif