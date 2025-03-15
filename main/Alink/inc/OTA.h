#ifndef _OTA_H_
#define _OTA_H_

#include "main.h"

#include "esp_crc.h"
#include "esp_flash_encrypt.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"

extern const char *TAG_OTA;  // 声明TAG_OTA

void user_download_recv_handler(void *handle, const aiot_mqtt_download_recv_t *packet, void *userdata);

/* 用户通过 aiot_ota_setopt() 注册的OTA消息处理回调, 如果SDK收到了OTA相关的MQTT消息, 会自动识别, 调用这个回调函数 */
void user_ota_recv_handler(void *ota_handle, aiot_ota_recv_t *ota_msg, void *userdata);

void *ota_task(void *args);

#endif





