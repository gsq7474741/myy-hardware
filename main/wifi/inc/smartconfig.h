#ifndef _SMARTCONFIG_H_
#define _SMARTCONFIG_H_

/* 宏定义WiFi更新标识 */
#define MY_WIFI_SAVE_FLAG    1994           /* 用于判断wifi是否经过配置的标志 */
#define NVS_WIFI_INFO_HANDLE "my_wifi_info" /* 用于读取nvs的命名空间 */
#define RETRY_CONNECT_TIME   20             /* wifi重连失败次数 */

#endif