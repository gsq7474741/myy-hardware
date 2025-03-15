#ifndef _WIFI_H_
#define _WIFI_H_

#include <stdlib.h>

#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "main.h"




/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT           = BIT0;// 连接信号
static const int ESPTOUCH_DONE_BIT       = BIT1;// 配网结束
static const int WIFI_CONFIGURED_BIT     = BIT2;// 网络是否配置过
static const int WIFI_NOT_CONFIGURED_BIT = BIT3;// 网络否配置过

extern const char *TAG_WIFI;  // 声明TAG_WIFI
extern bool wifi_connected;  // 声明 wifi_connected 变量
/* FreeRTOS event group to signal when we are connected and ready to make a request */
extern EventGroupHandle_t s_wifi_event_group; // 声明事件组

// 函数声明
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init(void);
void smartconfig_task(void *args);

#endif





