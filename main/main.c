/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "aiot_state_api.h"
#include "aiot_sysdep_api.h"
#include "aiot_mqtt_api.h"
#include "aiot_dm_api.h"

#include "aiot_ota_api.h"
#include "aiot_mqtt_download_api.h"


char *cur_version = "1.0.1"; // 固件更新时将此版本号更改为跟阿里云上面设置的一样的格式即可例如：“1.0.0”、“1.0.1”

/*485模块*/

#include "driver/uart.h"
#include "esp_system.h"
#include "driver/gpio.h"

typedef struct
{
    float lightValue;                    // 光照强度，单位 Lux
    float environment_moisture_float;    // 环境湿度，单位 %
    float environment_temperature_float; // 环境温度，单位 °C
    float solid_water_float;             // 土壤湿度，单位 %
    float solid_temp_float;              // 土壤温度，单位 °C
    float solid_EC_float;                // 土壤电导率，单位 μS/cm
    float solid_PH_float;                // 土壤 pH 值
} SensorData;

SensorData sensorData; // 创建一个 SensorData 类型的全局变量或静态变量

int waterswitch = 0;

#define TXD_PIN 17                   // RS485 TX pin
#define RXD_PIN 18                   // RS485 RX pin
#define RECEPT_ENABLE_PIN GPIO_NUM_1 // RS485 接收使能 pin
#define UART_PORT_NUM 1
#define BUF_SIZE (1024)

void RECEPT_ENABLE_PIN_Init()
{
    gpio_config_t io_config;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pin_bit_mask = 1ULL << RECEPT_ENABLE_PIN;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

// CRC16校验计算函数
unsigned int calculateCRC16(const uint8_t *data, uint8_t length)
{
    unsigned int crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// 发送Modbus命令
void sendModbusCommand(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value)
{
    uint8_t command[8];
    command[0] = address;                // 设备地址
    command[1] = functionCode;           // 功能码
    command[2] = registerAddress >> 8;   // 寄存器高字节
    command[3] = registerAddress & 0xFF; // 寄存器低字节
    command[4] = value >> 8;             // 数据高字节
    command[5] = value & 0xFF;           // 数据低字节

    unsigned int crc = calculateCRC16(command, 6); // 计算CRC
    command[6] = crc & 0xFF;                       // CRC低字节
    command[7] = crc >> 8;                         // CRC高字节
    gpio_set_level(RECEPT_ENABLE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    uart_write_bytes(UART_PORT_NUM, (const char *)command, sizeof(command)); // 发送命令
    uart_wait_tx_done(UART_PORT_NUM, 100 / portTICK_PERIOD_MS);
}

// 读取响应数据
bool readModbusResponse(uint8_t *buffer, uint8_t expectedLength, int timeout_ms)
{
    gpio_set_level(RECEPT_ENABLE_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    int len = uart_read_bytes(UART_PORT_NUM, buffer, expectedLength, pdMS_TO_TICKS(timeout_ms));
    return len == expectedLength;
}

// UART初始化
void init_rs485()
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void modbus_task(void *arg)
{

    RECEPT_ENABLE_PIN_Init();

    // 用于存储响应数据

    while (1)
    {
        // 光照传感器 0x01
        sendModbusCommand(0x01, 0x03, 0x0002, 0x0002); // 读取光照寄存器
        uint8_t readResponse_1[9];
        if (readModbusResponse(readResponse_1, 9, 1000))
        {
            uint16_t lightHigh = (readResponse_1[3] << 8) | readResponse_1[4];
            uint16_t lightLow = (readResponse_1[5] << 8) | readResponse_1[6];
            uint32_t lightFull = ((uint32_t)lightHigh << 16) | lightLow;
            sensorData.lightValue = lightFull / 1000.0;
            ESP_LOGI("Modbus", "地址 0x01 光照强度: %.3f Lux", sensorData.lightValue);
        }
        else
        {
            ESP_LOGW("Modbus", "读取地址 0x01 光照数据失败或无响应。");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 环境湿度和温度传感器 0x09
        sendModbusCommand(0x09, 0x03, 0x0000, 0x0002); // 读取湿度和温度寄存器
        uint8_t readResponse_2[9];
        if (readModbusResponse(readResponse_2, 9, 1000))
        {
            uint16_t moisture = (readResponse_2[3] << 8) | readResponse_2[4];
            uint16_t temperature = (readResponse_2[5] << 8) | readResponse_2[6];
            sensorData.environment_moisture_float = moisture / 10.0;
            sensorData.environment_temperature_float = temperature / 10.0;
            ESP_LOGI("Modbus", "地址 0x09 环境湿度: %.1f%% 环境温度: %.1f°C", sensorData.environment_moisture_float, sensorData.environment_temperature_float);
        }
        else
        {
            ESP_LOGW("Modbus", "读取地址 0x09 环境湿度/温度数据失败或无响应。");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 土壤传感器 0x03
        sendModbusCommand(0x03, 0x03, 0x0000, 0x0004); // 读取土壤寄存器
        uint8_t readResponse_3[13];
        if (readModbusResponse(readResponse_3, 13, 1000))
        {
            uint16_t water = (readResponse_3[3] << 8) | readResponse_3[4];
            uint16_t temp = (readResponse_3[5] << 8) | readResponse_3[6];
            uint16_t EC = (readResponse_3[7] << 8) | readResponse_3[8];
            uint16_t PH = (readResponse_3[9] << 8) | readResponse_3[10];
            sensorData.solid_water_float = water / 10.0;
            sensorData.solid_temp_float = temp / 10.0;
            sensorData.solid_EC_float = EC;
            sensorData.solid_PH_float = PH / 10.0;
            ESP_LOGI("Modbus", "地址 0x03 土壤湿度: %.1f%% 土壤温度: %.1f°C 土壤电导率: %.1f μS/cm 土壤PH: %.1f", sensorData.solid_water_float, sensorData.solid_temp_float, sensorData.solid_EC_float, sensorData.solid_PH_float);
        }
        else
        {
            ESP_LOGW("Modbus", "读取地址 0x03 土壤数据失败或无响应。");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/********************************************************* */

// 电磁阀
#define Solenoid_valves_PIN GPIO_NUM_2 // RS485 接收使能 pin
void Solenoid_valves_init(void)
{
    gpio_config_t io_config;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pin_bit_mask = 1ULL << Solenoid_valves_PIN;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

// 测试
void led38_valves_init(void)
{
    gpio_config_t io_config;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pin_bit_mask = 1ULL << GPIO_NUM_38;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

void Solenoid_valves_open(void)
{
    gpio_set_level(Solenoid_valves_PIN, 1);
}

void Solenoid_valves_close(void)
{
    gpio_set_level(Solenoid_valves_PIN, 0);
}

void Solenoid_contural(void)
{
    while (1)
    {
        if (waterswitch == 1)
        {
            gpio_set_level(Solenoid_valves_PIN, 1);
        }
        else
        {
            gpio_set_level(Solenoid_valves_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*************************************************************** */

#ifndef _SMARTCONFIG_H_
#define _SMARTCONFIG_H_

/* 宏定义WiFi更新标识 */
#define MY_WIFI_SAVE_FLAG 1994              /* 用于判断wifi是否经过配置的标志 */
#define NVS_WIFI_INFO_HANDLE "my_wifi_info" /* 用于读取nvs的命名空间 */
#define RETRY_CONNECT_TIME 20               /* wifi重连失败次数 */

void wifi_smartconfig_net_init(void);

#endif // !_SMARTCONFIG_H_

/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_eap_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;           // 连接信号
static const int ESPTOUCH_DONE_BIT = BIT1;       // 配网结束
static const int WIFI_CONFIGURED_BIT = BIT2;     // 网络是否配置过
static const int WIFI_NOT_CONFIGURED_BIT = BIT3; // 网络否配置过
static const char *TAG = "MAIN";                 // LOG 头
bool link_wifi = false;

static void smartconfig_example_task(void *parm);

/* 事件响应函数 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    static int retry_num = 0; /* 记录wifi重连次数 */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        EventBits_t uxBits;
        uxBits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONFIGURED_BIT | WIFI_NOT_CONFIGURED_BIT, true, false, portMAX_DELAY);
        if (uxBits & WIFI_CONFIGURED_BIT)
        {
            esp_wifi_connect();
            ESP_LOGI(TAG, "get WIFI_EVENT_STA_START , go ---> esp_wifi_connect .");
        }
        else
        {
            xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
            ESP_LOGI(TAG, "get WIFI_EVENT_STA_START , go ---> smartconfig_example_task .");
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
        link_wifi = true;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        retry_num++;
        ESP_LOGI(TAG, "retry to connect to the AP %d times. \n", retry_num);
        if (retry_num == RETRY_CONNECT_TIME) /* WiFi重连次数等于10 */
        {
            nvs_flash_erase_partition(NVS_WIFI_INFO_HANDLE);
            ESP_LOGI(TAG, "!!! retry connect num is enough , now retry smartconfig");
            esp_restart();
            // 重新配网
        }
        /* 清除WiFi连接成功标志位 */
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        // 重新联网次数清零
        retry_num = 0;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;                     /* 获取IP地址信息*/
        ESP_LOGI(TAG, "wifi is connected ip:%d.%d.%d.%d ", IP2STR(&event->ip_info.ip)); /* 打印ip地址*/
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "Scan done");
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
    {
        ESP_LOGI(TAG, "Found channel");
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        char ssid[33] = {0};
        char password[65] = {0};
        uint8_t rvd_data[33] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true)
        {
            ESP_LOGI(TAG, "Set MAC address of target AP: " MACSTR " ", MAC2STR(evt->bssid));
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
#endif

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2)
        {
            ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i = 0; i < 33; i++)
            {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }
        /* 将得到的WiFi名称和密码存入NVS*/
        nvs_handle nvs_my_wifi_info_handler;
        ESP_ERROR_CHECK(nvs_open(NVS_WIFI_INFO_HANDLE, NVS_READWRITE, &nvs_my_wifi_info_handler));
        ESP_ERROR_CHECK(nvs_set_u32(nvs_my_wifi_info_handler, "wifi_save_flag", MY_WIFI_SAVE_FLAG));
        ESP_ERROR_CHECK(nvs_set_str(nvs_my_wifi_info_handler, "wifi_ssid", (const char *)ssid));
        ESP_ERROR_CHECK(nvs_set_str(nvs_my_wifi_info_handler, "wifi_passwd", (const char *)password));
        ESP_ERROR_CHECK(nvs_commit(nvs_my_wifi_info_handler)); /* 提交 */
        nvs_close(nvs_my_wifi_info_handler);                   /* 关闭 */
        ESP_LOGI(TAG, "smartconfig save wifi info to NVS .");
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
    {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void initialise_wifi(void)
{
    /* 定义一个NVS操作句柄 */
    nvs_handle nvs_my_wifi_info_handler;
    uint32_t my_wifi_save_flag = 0;                   // nvs 存储标志
    ESP_ERROR_CHECK(esp_netif_init());                // 初始化协议栈基于TCP/IP
    s_wifi_event_group = xEventGroupCreate();         // 创建事件组
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 创建默认的事件组循环

    // 创建默认 WIFI STA。如果出现任何初始化错误，此 API 将中止。
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // 基于默认参数初始化wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件关联事件响应函数
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // 设置wifi为sta模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 打开一个NVS命名空间 */
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_INFO_HANDLE, NVS_READWRITE, &nvs_my_wifi_info_handler));

    // 读取wifi信息存储标志位
    nvs_get_u32(nvs_my_wifi_info_handler, "wifi_save_flag", &my_wifi_save_flag);

    // 存在标志位 证明记录过wifi配网信息
    if (MY_WIFI_SAVE_FLAG == my_wifi_save_flag)
    {
        wifi_config_t wifi_config;
        char *ssid = malloc(sizeof(wifi_config.sta.ssid));
        char *password = malloc(sizeof(wifi_config.sta.password));
        size_t required_size = 0;

        ESP_LOGI(TAG, "wifi info is Already exists,direct networking.");
        required_size = sizeof(wifi_config.sta.ssid); /* 从NVS中获取ssid */
        ESP_ERROR_CHECK(nvs_get_str(nvs_my_wifi_info_handler, "wifi_ssid", ssid, &required_size));
        required_size = sizeof(wifi_config.sta.password); /* 从NVS中获取ssid */
        ESP_ERROR_CHECK(nvs_get_str(nvs_my_wifi_info_handler, "wifi_passwd", password, &required_size));
        ESP_ERROR_CHECK(nvs_commit(nvs_my_wifi_info_handler)); /* 提交 */
        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
        ESP_LOGI(TAG, "IN NVS SSID:%s", ssid);
        ESP_LOGI(TAG, "IN NVS PASSWORD:%s", password);
        free(ssid);
        free(password);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONFIGURED_BIT);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "wifi info in nvs is exists,connect directlly.");
    }
    else
    {
        xEventGroupSetBits(s_wifi_event_group, WIFI_NOT_CONFIGURED_BIT);
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "wifi info in nvs is not exists,start smartconfig.");
    }
    nvs_close(nvs_my_wifi_info_handler); /* 关闭 */
}

static void smartconfig_example_task(void *parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1)
    {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
            link_wifi = true;
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

void wifi_smartconfig_net_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    // wifi配置
    initialise_wifi();
}

// #include "esp_eap_client.h"
// #include "esp_system.h"
// #include "esp_netif.h"
// #include "esp_smartconfig.h"
// #include "esp_mac.h"

// /* FreeRTOS event group to signal when we are connected & ready to make a request */
// static EventGroupHandle_t s_wifi_event_group;
// bool link_wifi = false;

// /* The event group allows multiple bits for each event,
//    but we only care about one event - are we connected
//    to the AP with an IP? */
// static const int CONNECTED_BIT = BIT0;
// static const int ESPTOUCH_DONE_BIT = BIT1;
// static const char *TAG = "smartconfig_example";

// static void smartconfig_example_task(void * parm);

// static void event_handler(void* arg, esp_event_base_t event_base,
//                                 int32_t event_id, void* event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         esp_wifi_connect();
//         xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
//     } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
//         ESP_LOGI(TAG, "Scan done");
//     } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
//         ESP_LOGI(TAG, "Found channel");
//     } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
//         ESP_LOGI(TAG, "Got SSID and password");

//         smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
//         wifi_config_t wifi_config;
//         uint8_t ssid[33] = { 0 };
//         uint8_t password[65] = { 0 };
//         uint8_t rvd_data[33] = { 0 };

//         bzero(&wifi_config, sizeof(wifi_config_t));
//         memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
//         memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

// #ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
//         wifi_config.sta.bssid_set = evt->bssid_set;
//         if (wifi_config.sta.bssid_set == true) {
//             ESP_LOGI(TAG, "Set MAC address of target AP: "MACSTR" ", MAC2STR(evt->bssid));
//             memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
//         }
// #endif

//         memcpy(ssid, evt->ssid, sizeof(evt->ssid));
//         memcpy(password, evt->password, sizeof(evt->password));
//         ESP_LOGI(TAG, "SSID:%s", ssid);
//         ESP_LOGI(TAG, "PASSWORD:%s", password);
//         if (evt->type == SC_TYPE_ESPTOUCH_V2) {
//             ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
//             ESP_LOGI(TAG, "RVD_DATA:");
//             for (int i=0; i<33; i++) {
//                 printf("%02x ", rvd_data[i]);
//             }
//             printf("\n");
//         }

//         ESP_ERROR_CHECK( esp_wifi_disconnect() );
//         ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
//         esp_wifi_connect();
//     } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
//         xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
//     }
// }

// static void initialise_wifi(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     s_wifi_event_group = xEventGroupCreate();
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
//     assert(sta_netif);

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

//     ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
//     ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
//     ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

//     ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//     ESP_ERROR_CHECK( esp_wifi_start() );
// }

// static void smartconfig_example_task(void * parm)
// {

//     EventBits_t uxBits;
//     ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS) );
//     smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
//     while (1) {
//         uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
//         if(uxBits & CONNECTED_BIT) {
//             ESP_LOGI(TAG, "WiFi Connected to ap");
//             link_wifi = true;
//         }
//         if(uxBits & ESPTOUCH_DONE_BIT) {
//             ESP_LOGI(TAG, "smartconfig over");
//             esp_smartconfig_stop();

//         }

//     }
// }

/*
 * 这个例程适用于`Linux`这类支持pthread的POSIX设备, 它演示了用SDK配置MQTT参数并建立连接, 之后创建2个线程
 *
 * + 一个线程用于保活长连接
 * + 一个线程用于接收消息, 并在有消息到达时进入默认的数据回调, 在连接状态变化时进入事件回调
 *
 * 需要用户关注或修改的部分, 已经用 TODO 在注释中标明
 *
 */

/* TODO: 替换为自己设备的三元组 */
// 产品密钥，设备名称，设备密钥

char *product_key = "ia66OXimz4V";
char *device_name = "nanhai_test_1";
char *device_secret = "1c5cc55678e5bf874b4462cf8a739856";

/* 位于portfiles/aiot_port文件夹下的系统适配函数集合 */
extern aiot_sysdep_portfile_t g_aiot_sysdep_portfile;

/* 位于external/ali_ca_cert.c中的服务器证书 */
extern const char *ali_ca_cert;
void *g_ota_handle = NULL;
void *g_dl_handle = NULL;
void *ota_handle = NULL;
uint32_t g_firmware_size = 0;

static pthread_t g_mqtt_process_thread;
static pthread_t g_mqtt_recv_thread;
static pthread_t ota_thread;

static uint8_t g_mqtt_process_thread_running = 0;
static uint8_t g_mqtt_recv_thread_running = 0;
static uint8_t ota_running = 0;

/* TODO: 如果要关闭日志, 就把这个函数实现为空, 如果要减少日志, 可根据code选择不打印
 *
 * 例如: [1577589489.033][LK-0317] mqtt_basic_demo&a13FN5TplKq
 *
 * 上面这条日志的code就是0317(十六进制), code值的定义见core/aiot_state_api.h
 *
 */

/* 日志回调函数, SDK的日志会从这里输出 */
int32_t demo_state_logcb(int32_t code, char *message)
{
    printf("%s", message);
    return 0;
}

/* MQTT事件回调函数, 当网络连接/重连/断开时被触发, 事件定义见core/aiot_mqtt_api.h */
void demo_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata)
{
    switch (event->type)
    {
    /* SDK因为用户调用了aiot_mqtt_connect()接口, 与mqtt服务器建立连接已成功 */
    case AIOT_MQTTEVT_CONNECT:
    {
        printf("AIOT_MQTTEVT_CONNECT\n");
        /* TODO: 处理SDK建连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

        /* SDK因为网络状况被动断连后, 自动发起重连已成功 */
    case AIOT_MQTTEVT_RECONNECT:
    {
        printf("AIOT_MQTTEVT_RECONNECT\n");
        /* TODO: 处理SDK重连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

        /* SDK因为网络的状况而被动断开了连接, network是底层读写失败, heartbeat是没有按预期得到服务端心跳应答 */
    case AIOT_MQTTEVT_DISCONNECT:
    {
        char *cause = (event->data.disconnect == AIOT_MQTTDISCONNEVT_NETWORK_DISCONNECT) ? ("network disconnect") : ("heartbeat disconnect");
        printf("AIOT_MQTTEVT_DISCONNECT: %s\n", cause);
        /* TODO: 处理SDK被动断连, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    default:
    {
    }
    }
}

/* MQTT默认消息处理回调, 当SDK从服务器收到MQTT消息时, 且无对应用户回调处理时被调用 */
void demo_mqtt_default_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata)
{
    switch (packet->type)
    {
    case AIOT_MQTTRECV_HEARTBEAT_RESPONSE:
    {
        printf("heartbeat response\n");
        /* TODO: 处理服务器对心跳的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_SUB_ACK:
    {
        printf("suback, res: -0x%04lX, packet id: %d, max qos: %d\n",
               -packet->data.sub_ack.res, packet->data.sub_ack.packet_id, packet->data.sub_ack.max_qos);
        /* TODO: 处理服务器对订阅请求的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_PUB:
    {
        printf("pub, qos: %d, topic: %.*s\n", packet->data.pub.qos, packet->data.pub.topic_len, packet->data.pub.topic);
        // printf("pub, payload: %.*s\n", packet->data.pub.payload_len, packet->data.pub.payload);
        printf("pub, payload: %.*s\n", (int)packet->data.pub.payload_len, packet->data.pub.payload);
        /* TODO: 处理服务器下发的业务报文 */
    }
    break;

    case AIOT_MQTTRECV_PUB_ACK:
    {
        printf("puback, packet id: %d\n", packet->data.pub_ack.packet_id);
        /* TODO: 处理服务器对QoS1上报消息的回应, 一般不处理 */
    }
    break;

    default:
    {
    }
    }
}

/*****************************************OTA********************************************* */

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "aiot_mqtt_api.h"
#include "esp_flash_encrypt.h"
#include "esp_partition.h"
#include "esp_crc.h"

esp_err_t err;
/* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
esp_ota_handle_t update_handle = 0;
const esp_partition_t *update_partition = NULL;

const esp_partition_t *configured = NULL;
const esp_partition_t *running = NULL;



static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1)
    {
        ;
    }
}

/* 下载收包回调, 用户调用 aiot_download_recv() 后, SDK收到数据会进入这个函数, 把下载到的数据交给用户 */
void user_download_recv_handler(void *handle, const aiot_mqtt_download_recv_t *packet, void *userdata)
{
    uint32_t data_buffer_len = 0;

    /* 目前只支持packet->type为AIOT_MDRECV_DATA_RESP的情况 */
    if (!packet || AIOT_MDRECV_DATA_RESP != packet->type)
    {
        return;
    }


    /* 应在此实现文件本地固化的操作 */

    err = esp_ota_write(update_handle, (const void *)packet->data.data_resp.data, packet->data.data_resp.data_size);
    if (err != ESP_OK)
    {
        esp_ota_abort(update_handle);
        task_fatal_error();
    }
    data_buffer_len = packet->data.data_resp.data_size;
    printf("download %03ld%% done, +%ld bytes\r\n", packet->data.data_resp.percent, data_buffer_len);

}

/* 用户通过 aiot_ota_setopt() 注册的OTA消息处理回调, 如果SDK收到了OTA相关的MQTT消息, 会自动识别, 调用这个回调函数 */
void user_ota_recv_handler(void *ota_handle, aiot_ota_recv_t *ota_msg, void *userdata)
{
    uint32_t request_size = 10 * 1024;
    switch (ota_msg->type)
    {
    case AIOT_OTARECV_FOTA:
    {
        if (NULL == ota_msg->task_desc || ota_msg->task_desc->protocol_type != AIOT_OTA_PROTOCOL_MQTT)
        {
            break;
        }
        printf("OTA target firmware version: %s, size: %lu Bytes\r\n", ota_msg->task_desc->version,
               ota_msg->task_desc->size_total);

        void *md_handler = aiot_mqtt_download_init();

        printf("wait aiot_mqtt_download_setopt\r\n");
        sleep(5);
        aiot_mqtt_download_setopt(md_handler, AIOT_MDOPT_TASK_DESC, ota_msg->task_desc);
        /* 设置下载一包的大小，对于资源受限设备可以调整该值大小 */
        aiot_mqtt_download_setopt(md_handler, AIOT_MDOPT_DATA_REQUEST_SIZE, &request_size);
        aiot_mqtt_download_setopt(md_handler, AIOT_MDOPT_RECV_HANDLE, user_download_recv_handler);

        printf("end aiot_mqtt_download_setopt\r\n");
        sleep(5);

        g_dl_handle = md_handler;
    }
    default:
        break;
    }
}

void ota_task(void)
{
    ESP_LOGI(TAG, "Starting OTA example task");

    configured = esp_ota_get_boot_partition();
    running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08" PRIx32 ", but running from offset 0x%08" PRIx32,
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08" PRIx32 ")",
             running->type, running->subtype, running->address);

    // 获取目标分区
    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        esp_ota_abort(update_handle);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    while (1)
    {
        if (g_dl_handle != NULL)
        {
            int32_t res = aiot_mqtt_download_process(g_dl_handle);

            if (STATE_MQTT_DOWNLOAD_SUCCESS == res)
            {
                /* 升级成功，可在此处重启并且上报新的版本号 */
                printf("mqtt download ota success \r\n");
                err = esp_ota_end(update_handle);
                if (err != ESP_OK)
                {
                    if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                    {
                        ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
                    }
                    task_fatal_error();
                }
                err = esp_ota_set_boot_partition(update_partition);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                    task_fatal_error();
                }
                ESP_LOGI(TAG, "Prepare to restart system!");
                aiot_mqtt_download_deinit(&g_dl_handle);
                esp_restart();
                return;
                // aiot_mqtt_download_deinit(&g_dl_handle);
                // break;
            }
            else if (STATE_MQTT_DOWNLOAD_FAILED_RECVERROR == res || STATE_MQTT_DOWNLOAD_FAILED_TIMEOUT == res || STATE_MQTT_DOWNLOAD_FAILED_MISMATCH == res)
            {
                printf("mqtt download ota failed \r\n");
                aiot_mqtt_download_deinit(&g_dl_handle);
                break;
            }
        }
        sleep(1);
    }
}

/* 执行aiot_mqtt_process的线程, 包含心跳发送和QoS1消息重发 */
void *demo_mqtt_process_thread(void *args)
{
    int32_t res = STATE_SUCCESS;

    while (g_mqtt_process_thread_running)
    {
        res = aiot_mqtt_process(args);
        if (res == STATE_USER_INPUT_EXEC_DISABLED)
        {
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

    while (g_mqtt_recv_thread_running)
    {
        res = aiot_mqtt_recv(args);
        if (res < STATE_SUCCESS)
        {
            if (res == STATE_USER_INPUT_EXEC_DISABLED)
            {
                break;
            }
            sleep(1);
        }
    }
    return NULL;
}

//******************************************************************************************************************************************
/*处理服务器下发的消息*/
/*******************************************************************************/

// 这个函数用于处理服务器发送给客户端的回复消息。当客户端向服务器发送请求（例如，查询设备状态、更新配置等）
// 服务器会对这些请求做出响应，回复消息会包含请求的结果和其他相关信息。
static void demo_dm_recv_generic_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_generic_reply msg_id = %lu, code = %lu, data = %.*s, message = %.*s\r\n",
           recv->data.generic_reply.msg_id,
           recv->data.generic_reply.code,
           (int)recv->data.generic_reply.data_len,
           recv->data.generic_reply.data,
           (int)recv->data.generic_reply.message_len,
           recv->data.generic_reply.message);
}

// 处理服务器下发的属性设置，比如温度设置
static void demo_dm_recv_property_set(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_property_set msg_id = %ld, params = %.*s\r\n",
           (unsigned long)recv->data.property_set.msg_id,
           (int)recv->data.property_set.params_len,
           recv->data.property_set.params);
    // 设备云端在线调试控制电磁阀

    // 临时添加终止符
    char *temp_params = malloc(recv->data.property_set.params_len + 1);
    if (temp_params == NULL)
    {
        printf("Memory allocation failed\n");
        return;
    }
    memcpy(temp_params, recv->data.property_set.params, recv->data.property_set.params_len);
    temp_params[recv->data.property_set.params_len] = '\0';

    if (strcmp(temp_params, "{\"WaterOutletSwitch\":0}") == 0)
    {
        // Solenoid_valves_close();
        waterswitch = 0;
        gpio_set_level(Solenoid_valves_PIN, 0);
        gpio_set_level(GPIO_NUM_38, 0);
        printf("电磁阀关闭\r\n");
        // WaterOutletSwitch_send_property_post(dm_handle,0);
    }
    if (strcmp(temp_params, "{\"WaterOutletSwitch\":1}") == 0)
    {
        // Solenoid_valves_open();
        waterswitch = 1;
        gpio_set_level(Solenoid_valves_PIN, 1);
        gpio_set_level(GPIO_NUM_38, 1);
        printf("电磁阀打开\r\n");
        // WaterOutletSwitch_send_property_post(dm_handle,1);
    }

    /* TODO: 以下代码演示如何对来自云平台的属性设置指令进行应答, 用户可取消注释查看演示效果 */
    /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_PROPERTY_SET_REPLY;
        msg.data.property_set_reply.msg_id = recv->data.property_set.msg_id;
        msg.data.property_set_reply.code = 200;
        msg.data.property_set_reply.data = "{}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0) {
            printf("aiot_dm_send failed\r\n");
        }
    }
    */
}

// 可设置设备的开关状态，用于处理设备管理（Device Management，DM）接收到的异步服务调用请求。
static void demo_dm_recv_async_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_async_service_invoke msg_id = %ld, service_id = %s, params = %.*s\r\n",
           (unsigned long)recv->data.async_service_invoke.msg_id,
           recv->data.async_service_invoke.service_id,
           (int)recv->data.async_service_invoke.params_len,
           recv->data.async_service_invoke.params);

    /* TODO: 以下代码演示如何对来自云平台的异步服务调用进行应答, 用户可取消注释查看演示效果
     *
     * 注意: 如果用户在回调函数外进行应答, 需要自行保存msg_id, 因为回调函数入参在退出回调函数后将被SDK销毁, 不可以再访问到
     */

    /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_ASYNC_SERVICE_REPLY;
        msg.data.async_service_reply.msg_id = recv->data.async_service_invoke.msg_id;
        msg.data.async_service_reply.code = 200;
        msg.data.async_service_reply.service_id = "ToggleLightSwitch";
        msg.data.async_service_reply.data = "{\"dataA\": 20}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0) {
            printf("aiot_dm_send failed\r\n");
        }
    }
    */
}
static void demo_dm_recv_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_sync_service_invoke msg_id = %ld, rrpc_id = %s, service_id = %s, params = %.*s\r\n",
           (unsigned long)recv->data.sync_service_invoke.msg_id,
           recv->data.sync_service_invoke.rrpc_id,
           recv->data.sync_service_invoke.service_id,
           (int)recv->data.sync_service_invoke.params_len,
           recv->data.sync_service_invoke.params);

    /* TODO: 以下代码演示如何对来自云平台的同步服务调用进行应答, 用户可取消注释查看演示效果
     *
     * 注意: 如果用户在回调函数外进行应答, 需要自行保存msg_id和rrpc_id字符串, 因为回调函数入参在退出回调函数后将被SDK销毁, 不可以再访问到
     */

    /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_SYNC_SERVICE_REPLY;
        msg.data.sync_service_reply.rrpc_id = recv->data.sync_service_invoke.rrpc_id;
        msg.data.sync_service_reply.msg_id = recv->data.sync_service_invoke.msg_id;
        msg.data.sync_service_reply.code = 200;
        msg.data.sync_service_reply.service_id = "SetLightSwitchTimer";
        msg.data.sync_service_reply.data = "{}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0) {
            printf("aiot_dm_send failed\r\n");
        }
    }
    */
}

static void demo_dm_recv_raw_data(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_data raw data len = %lu\r\n", recv->data.raw_data.data_len);
    /* TODO: 以下代码演示如何发送二进制格式数据, 若使用需要有相应的数据透传脚本部署在云端 */
    /*
    {
        aiot_dm_msg_t msg;
        uint8_t raw_data[] = {0x01, 0x02};

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_RAW_DATA;
        msg.data.raw_data.data = raw_data;
        msg.data.raw_data.data_len = sizeof(raw_data);
        aiot_dm_send(dm_handle, &msg);
    }
    */
}

// 同步服务调用请求（raw sync service invoke）是一种在客户端和服务器之间进行数据交互的方式，特别是在物联网（IoT）场景中，
// 这种调用方式允许客户端发送请求并在收到服务器响应之前阻塞等待。
// 这种方式保证了请求的顺序执行，并且在处理结果之前不会继续执行后续代码。
static void demo_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_sync_service_invoke raw sync service rrpc_id = %s, data_len = %lu\r\n",
           recv->data.raw_service_invoke.rrpc_id,
           recv->data.raw_service_invoke.data_len);
}

static void demo_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_data_reply receive reply for up_raw msg, data len = %lu\r\n", recv->data.raw_data.data_len);
    /* TODO: 用户处理下行的二进制数据, 位于recv->data.raw_data.data中 */
}

/* 用户数据接收处理回调函数 */
static void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_handler, type = %d\r\n", recv->type);

    switch (recv->type)
    {

    /* 属性上报, 事件上报, 获取期望属性值或者删除期望属性值的应答 */
    case AIOT_DMRECV_GENERIC_REPLY:
    {
        demo_dm_recv_generic_reply(dm_handle, recv, userdata);
    }
    break;

        /* 属性设置 */
    case AIOT_DMRECV_PROPERTY_SET:
    {
        demo_dm_recv_property_set(dm_handle, recv, userdata);
    }
    break;

        /* 异步服务调用 */
    case AIOT_DMRECV_ASYNC_SERVICE_INVOKE:
    {
        demo_dm_recv_async_service_invoke(dm_handle, recv, userdata);
    }
    break;

        /* 同步服务调用 */
    case AIOT_DMRECV_SYNC_SERVICE_INVOKE:
    {
        demo_dm_recv_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

        /* 下行二进制数据 */
    case AIOT_DMRECV_RAW_DATA:
    {
        demo_dm_recv_raw_data(dm_handle, recv, userdata);
    }
    break;

        /* 二进制格式的同步服务调用, 比单纯的二进制数据消息多了个rrpc_id */
    case AIOT_DMRECV_RAW_SYNC_SERVICE_INVOKE:
    {
        demo_dm_recv_raw_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

        /* 上行二进制数据后, 云端的回复报文 */
    case AIOT_DMRECV_RAW_DATA_REPLY:
    {
        demo_dm_recv_raw_data_reply(dm_handle, recv, userdata);
    }
    break;

    default:
        break;
    }
}

/* 属性上报函数演示 */
// 发送单个属性数据
int32_t demo_send_property_post(void *dm_handle, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

// 发送多个属性数据
int32_t demo_send_property_batch_post(void *dm_handle, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_BATCH_POST;
    msg.data.property_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 事件上报函数演示 */
// 例如开/关等事件
int32_t demo_send_event_post(void *dm_handle, char *event_id, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_EVENT_POST;
    msg.data.event_post.event_id = event_id;
    msg.data.event_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/*各属性 上报函数*/

// CurrentTemperature
void CurrentTemperature_send_property_post(void *dm_handle, float CurrentTemperature)
{
    char jsonBuffer[128];                                                                           // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"CurrentTemperature\": %.1f}", CurrentTemperature); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                                                 // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// WaterOutletSwitch
void WaterOutletSwitch_send_property_post(void *dm_handle, int WaterOutletSwitch)
{
    char jsonBuffer[128];                                                                       // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"WaterOutletSwitch\": %d}", WaterOutletSwitch); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                                             // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤钾 SoilK
void SoilK_send_property_post(void *dm_handle, float SoilK)
{
    char jsonBuffer[128];                                                 // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilK\": %.1f}", SoilK); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                       // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤磷 SoilP
void SoilP_send_property_post(void *dm_handle, float SoilP)
{
    char jsonBuffer[128];                                                 // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilP\": %.1f}", SoilP); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                       // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤氮 SoilN
void SoilN_send_property_post(void *dm_handle, float SoilN)
{
    char jsonBuffer[128];                                                 // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilN\": %.1f}", SoilN); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                       // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤温度 SoilTemperature
void SoilTemperature_send_property_post(void *dm_handle, float SoilTemperature)
{
    char jsonBuffer[128];                                                                     // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilTemperature\": %.1f}", SoilTemperature); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                                           // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤湿度 SoilHumidity
void SoilHumidity_send_property_post(void *dm_handle, float SoilHumidity)
{
    char jsonBuffer[128];                                                               // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilHumidity\": %.1f}", SoilHumidity); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                                     // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤PH值 SoilPH
void SoilPH_send_property_post(void *dm_handle, float SoilPH)
{
    char jsonBuffer[128];                                                   // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilPH\": %.1f}", SoilPH); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                         // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 土壤EC值 SoilEC
void SoilEC_send_property_post(void *dm_handle, float SoilEC)
{
    char jsonBuffer[128];                                                   // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"SoilEC\": %.1f}", SoilEC); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                         // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 相对湿度 RelativeHumidity
void RelativeHumidity_send_property_post(void *dm_handle, float RelativeHumidity)
{
    char jsonBuffer[128];                                                                       // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"RelativeHumidity\": %.1f}", RelativeHumidity); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                                             // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

// 光照强度 LightLux
void LightLux_send_property_post(void *dm_handle, float LightLux)
{
    char jsonBuffer[128];                                                       // 定义一个足够大的缓冲区来存放 JSON 字符串
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"LightLux\": %.1f}", LightLux); // 使用 snprintf 格式化字符串
    demo_send_property_post(dm_handle, jsonBuffer);                             // 使用格式化后的字符串作为参数调用 demo_send_property_post
}

/********************************************************************************************************************************************************/

/*****************************************************************************************************************/

int linkkit_main(void)
{

    /*************
     * mptt服务器初始化
     * ***************/

    void *dm_handle = NULL;

    uint8_t post_reply = 1;

    // 设备管理模块的消息接收处理器为 demo_dm_recv_handler
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *)demo_dm_recv_handler);
    // 设置了是否需要云端回复设备发送的POST请求
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *)&post_reply);

    int32_t res = STATE_SUCCESS;
    void *mqtt_handle = NULL;
    char *url = "iot-06z009wo9u7aniw.mqtt.iothub.aliyuncs.com"; /* 阿里云平台上海站点的域名后缀 */
    char host[100] = {0};                                       /* 用这个数组拼接设备连接的云平台站点全地址, 规则是 ${productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com */
    uint16_t port = 1883;                                       /* 无论设备是否使用TLS连接阿里云平台, 目的端口都是443 */
    aiot_sysdep_network_cred_t cred;                            /* 安全凭据结构体, 如果要用TLS, 这个结构体中配置CA证书等参数 */

    /* 配置SDK的底层依赖 */
    aiot_sysdep_set_portfile(&g_aiot_sysdep_portfile);
    /* 配置SDK的日志输出 */
    aiot_state_set_logcb(demo_state_logcb);

    /* 创建SDK的安全凭据, 用于建立TLS连接 */
    memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
    cred.option = AIOT_SYSDEP_NETWORK_CRED_SVRCERT_CA; /* 使用RSA证书校验MQTT服务端 */
    cred.max_tls_fragment = 16384;                     /* 最大的分片长度为16K, 其它可选值还有4K, 2K, 1K, 0.5K */
    cred.sni_enabled = 1;                              /* TLS建连时, 支持Server Name Indicator */
    cred.x509_server_cert = ali_ca_cert;               /* 用来验证MQTT服务端的RSA根证书 */
    cred.x509_server_cert_len = strlen(ali_ca_cert);   /* 用来验证MQTT服务端的RSA根证书长度 */

    /* 创建1个MQTT客户端实例并内部初始化默认参数 */
    mqtt_handle = aiot_mqtt_init();
    if (mqtt_handle == NULL)
    {
        printf("aiot_mqtt_init failed\n");
        return -1;
    }

    /* TODO: 如果以下代码不被注释, 则例程会用TCP而不是TLS连接云平台 */

    {
        memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
        cred.option = AIOT_SYSDEP_NETWORK_CRED_NONE;
    }

    // snprintf(host, 100, "%s.%s", product_key, url);
    snprintf(host, 100, "%s", url);
    /* 配置MQTT服务器地址 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_HOST, (void *)host);
    /* 配置MQTT服务器端口 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PORT, (void *)&port);
    /* 配置设备productKey */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PRODUCT_KEY, (void *)product_key);
    /* 配置设备deviceName */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_NAME, (void *)device_name);
    /* 配置设备deviceSecret */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_SECRET, (void *)device_secret);
    /* 配置网络连接的安全凭据, 上面已经创建好了 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_NETWORK_CRED, (void *)&cred);
    /* 配置MQTT默认消息接收回调函数 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_RECV_HANDLER, (void *)demo_mqtt_default_recv_handler);
    /* 配置MQTT事件回调函数 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_EVENT_HANDLER, (void *)demo_mqtt_event_handler);

    /* 与MQTT例程不同的是, 这里需要增加创建OTA会话实例的语句 */
    ota_handle = aiot_ota_init();
    if (NULL == ota_handle)
    {
        return -1;
    }

    /* 用以下语句, 把OTA会话和MQTT会话关联起来 */
    aiot_ota_setopt(ota_handle, AIOT_OTAOPT_MQTT_HANDLE, mqtt_handle);
    /* 用以下语句, 设置OTA会话的数据接收回调, SDK收到OTA相关推送时, 会进入这个回调函数 */
    aiot_ota_setopt(ota_handle, AIOT_OTAOPT_RECV_HANDLER, user_ota_recv_handler);
    g_ota_handle = ota_handle;

    /*****************************
     * 客户端，设备初始化
     *****************************/

    /* 创建DATA-MODEL实例 */
    dm_handle = aiot_dm_init();
    if (dm_handle == NULL)
    {
        printf("aiot_dm_init failed");
        return -1;
    }
    /* 配置MQTT实例句柄 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_MQTT_HANDLE, mqtt_handle);
    /* 配置消息接收处理回调函数 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *)demo_dm_recv_handler);

    /* 配置是云端否需要回复post_reply给设备. 如果为1, 表示需要云端回复, 否则表示不回复 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *)&post_reply);

    /**************************************************************************************************/

    /* 与服务器建立MQTT连接 */
    res = aiot_mqtt_connect(mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        /* 尝试建立连接失败, 销毁MQTT实例, 回收资源 */
        aiot_mqtt_deinit(&mqtt_handle);
        printf("aiot_mqtt_connect failed: -0x%04lX\n", -res);
        goto exit;
    }

    /* 演示MQTT连接建立起来之后, 就可以上报当前设备的版本号了 */
    res = aiot_ota_report_version(ota_handle, cur_version);
    if (res < STATE_SUCCESS)
    {
        printf("report version failed, code is -0x%04lX\r\n", -res);
    }

    /* MQTT 订阅topic功能示例, 请根据自己的业务需求进行使用 */
    // 设备上报属性之后，云端对设备的回复，通过这种方式来保证数据更新到云端
    {
        char *sub_topic = "/sys/a13FN5TplKq/mqtt_basic_demo/thing/event/+/post_reply";

        res = aiot_mqtt_sub(mqtt_handle, sub_topic, NULL, 1, NULL);
        if (res < 0)
        {
            printf("aiot_mqtt_sub failed, res: -0x%04lX\n", -res);
            return -1;
        }
    }

    /* MQTT 发布消息功能示例, 请根据自己的业务需求进行使用 */
    // 云端给设备发送指令，如控制开关等
    {
        char *pub_topic = "/sys/a13FN5TplKq/mqtt_basic_demo/thing/event/property/post";
        char *pub_payload = "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"LightSwitch\":0}}";

        res = aiot_mqtt_pub(mqtt_handle, pub_topic, (uint8_t *)pub_payload, strlen(pub_payload), 0);
        if (res < 0)
        {
            printf("aiot_mqtt_sub failed, res: -0x%04lX\n", -res);
            return -1;
        }
    }

    /* 创建一个单独的线程, 专用于执行aiot_mqtt_process, 它会自动发送心跳保活, 以及重发QoS1的未应答报文 */
    g_mqtt_process_thread_running = 1;
    res = pthread_create(&g_mqtt_process_thread, NULL, demo_mqtt_process_thread, mqtt_handle);
    if (res < 0)
    {
        printf("pthread_create demo_mqtt_process_thread failed: %ld\n", res);
        return -1;
    }

    /* 创建一个单独的线程用于执行aiot_mqtt_recv, 它会循环收取服务器下发的MQTT消息, 并在断线时自动重连 */
    g_mqtt_recv_thread_running = 1;
    res = pthread_create(&g_mqtt_recv_thread, NULL, demo_mqtt_recv_thread, mqtt_handle);
    if (res < 0)
    {
        printf("pthread_create demo_mqtt_recv_thread failed: %ld\n", res);
        return -1;
    }

    ota_running = 1;
    res = pthread_create(&ota_thread, NULL, ota_task, NULL);
    if (res < 0)
    {
        printf("pthread_create ota_task failed: %ld\n", res);
        return -1;
    }

    /* 主循环进入休眠 */
    while (1)
    {
        WaterOutletSwitch_send_property_post(dm_handle, waterswitch);
        vTaskDelay(pdMS_TO_TICKS(1000));
        CurrentTemperature_send_property_post(dm_handle, sensorData.environment_temperature_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // SoilK_send_property_post(dm_handle,60.1);
        // SoilP_send_property_post(dm_handle,60.1);
        // SoilN_send_property_post(dm_handle,60.1);
        // SoilK_send_property_post(dm_handle,60.1);
        SoilTemperature_send_property_post(dm_handle, sensorData.solid_temp_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        SoilHumidity_send_property_post(dm_handle, sensorData.solid_water_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        SoilPH_send_property_post(dm_handle, sensorData.solid_PH_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        SoilEC_send_property_post(dm_handle, sensorData.solid_EC_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        RelativeHumidity_send_property_post(dm_handle, sensorData.environment_moisture_float);
        vTaskDelay(pdMS_TO_TICKS(1000));
        LightLux_send_property_post(dm_handle, sensorData.lightValue);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // demo_send_property_post(dm_handle, "{\"WaterOutletSwitch\": 0}");
        // demo_send_property_post(dm_handle, "{\"CurrentTemperature\": 37.5}");
        // demo_send_property_batch_post(dm_handle, "{\"properties\":{\"WaterOutletSwitch\": [ {\"value\":0,\"time\":1726475700683 }],\"CurrentTemperature\": [{\"value\": 30.9,\"time\":1726475700683 }]}}");

        sleep(1);
    }

exit:
    /* 停止收发动作 */
    g_mqtt_process_thread_running = 0;
    g_mqtt_recv_thread_running = 0;

    // 断开MQTT连接, 一般不会运行到这里
    res = aiot_mqtt_disconnect(mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        aiot_mqtt_deinit(&mqtt_handle);
        printf("aiot_mqtt_disconnect failed: -0x%04lX\n", -res);
        return -1;
    }

    //* 销毁MQTT实例, 一般不会运行到这里
    res = aiot_mqtt_deinit(&mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        printf("aiot_mqtt_deinit failed: -0x%04lX\n", -res);
        return -1;
    }

    /* 销毁OTA实例, 一般不会运行到这里 */
    aiot_ota_deinit(&ota_handle);

    g_mqtt_process_thread_running = 0;
    g_mqtt_recv_thread_running = 0;
    pthread_join(g_mqtt_process_thread, NULL);
    pthread_join(g_mqtt_recv_thread, NULL);

    return 0;
}

void app_main(void)
{

    // 初始化RS485 UART
    init_rs485();
    Solenoid_valves_init();
    led38_valves_init();
    // 创建Modbus任务
    xTaskCreate(modbus_task, "modbus_task", 8192, NULL, 10, NULL);
    // Solenoid_contural
    // xTaskCreate(Solenoid_contural, "Solenoid_contural", 8192, NULL, 9, NULL);

    // SmartConfig
    // nvs_flash_erase();
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();
    while (!link_wifi)
    {
        printf("no wifi\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* start linkkit mqtt */
    ESP_LOGI(TAG, "Start linkkit mqtt");
    linkkit_main();
}
