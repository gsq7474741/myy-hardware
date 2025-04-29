#include "smartconfig.h"
#include "wifi.h"

bool wifi_connected;  // 声明 wifi_connected 变量
EventGroupHandle_t s_wifi_event_group;  // 定义事件组句柄

void erase_wifi_config() {
	nvs_handle_t wifi_nvs_handle;
	esp_err_t err;

	// 打开NVS命名空间
	err = nvs_open(NVS_WIFI_INFO_HANDLE, NVS_READWRITE, &wifi_nvs_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_WIFI, "Failed to open NVS namespace: %s", esp_err_to_name(err));
		return;
	}

	// 删除特定的键值
	err = nvs_erase_key(wifi_nvs_handle, "wifi_save_flag");
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGE(TAG_WIFI, "Failed to erase wifi_save_flag: %s", esp_err_to_name(err));
	}

	err = nvs_erase_key(wifi_nvs_handle, "wifi_ssid");
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGE(TAG_WIFI, "Failed to erase wifi_ssid: %s", esp_err_to_name(err));
	}

	err = nvs_erase_key(wifi_nvs_handle, "wifi_passwd");
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGE(TAG_WIFI, "Failed to erase wifi_passwd: %s", esp_err_to_name(err));
	}

	// 提交更改
	nvs_commit(wifi_nvs_handle);

	// 关闭句柄
	nvs_close(wifi_nvs_handle);

	ESP_LOGI(TAG_WIFI, "WiFi配置信息已成功擦除");
}


/* WIFI重连事件响应函数 */
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	static int retry_num = 0; /* 记录wifi重连次数 */
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		EventBits_t uxBits;
		uxBits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONFIGURED_BIT | WIFI_NOT_CONFIGURED_BIT, true, false, portMAX_DELAY);
		if (uxBits & WIFI_CONFIGURED_BIT) {
			esp_wifi_connect();
			ESP_LOGI(TAG_WIFI, "get WIFI_EVENT_STA_START , go ---> esp_wifi_connect .");
		} else {
			xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
			ESP_LOGI(TAG_WIFI, "get WIFI_EVENT_STA_START , go ---> smartconfig_task .");
		}
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
		vTaskDelay(pdMS_TO_TICKS(3000));
		wifi_connected = true;
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		esp_wifi_connect();
		retry_num++;
		ESP_LOGI(TAG_WIFI, "retry to connect to the AP %d times. \n", retry_num);
		if (retry_num == RETRY_CONNECT_TIME) /* WiFi重连次数等于10 */
		{
			// 替换原来的擦除代码
			erase_wifi_config();
			ESP_LOGI(TAG_WIFI, "!!! retry connect num is enough , now retry smartconfig");

			// 在重启前增加一个小延迟，确保NVS操作完成
			vTaskDelay(pdMS_TO_TICKS(100));
			esp_restart();
		}
		/* 清除WiFi连接成功标志位 */
		xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		// 重新联网次数清零
		retry_num                = 0;
		ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;                    /* 获取IP地址信息*/
		ESP_LOGI(TAG_WIFI, "wifi is connected ip:%d.%d.%d.%d ", IP2STR(&event->ip_info.ip)); /* 打印ip地址*/
		xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
	} else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
		ESP_LOGI(TAG_WIFI, "Scan done");
	} else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
		ESP_LOGI(TAG_WIFI, "Found channel");
	} else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
		ESP_LOGI(TAG_WIFI, "Got SSID and password");

		smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *) event_data;
		wifi_config_t                      wifi_config;
		char                               ssid[33]     = {0};
		char                               password[65] = {0};
		uint8_t                            rvd_data[33] = {0};

		bzero(&wifi_config, sizeof(wifi_config_t));
		memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
		memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
		wifi_config.sta.bssid_set = evt->bssid_set;
		if (wifi_config.sta.bssid_set == true) {
			ESP_LOGI(TAG_WIFI, "Set MAC address of target AP: " MACSTR " ", MAC2STR(evt->bssid));
			memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
		}
#endif

		memcpy(ssid, evt->ssid, sizeof(evt->ssid));
		memcpy(password, evt->password, sizeof(evt->password));
		ESP_LOGI(TAG_WIFI, "SSID:%s", ssid);
		ESP_LOGI(TAG_WIFI, "PASSWORD:%s", password);
		if (evt->type == SC_TYPE_ESPTOUCH_V2) {
			ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
			ESP_LOGI(TAG_WIFI, "RVD_DATA:");
			for (int i = 0; i < 33; i++) {
				printf("%02x ", rvd_data[i]);
			}
			printf("\n");
		}
		/* 将得到的WiFi名称和密码存入NVS*/
		nvs_handle nvs_my_wifi_info_handler;
		ESP_ERROR_CHECK(nvs_open(NVS_WIFI_INFO_HANDLE, NVS_READWRITE, &nvs_my_wifi_info_handler));
		ESP_ERROR_CHECK(nvs_set_u32(nvs_my_wifi_info_handler, "wifi_save_flag", MY_WIFI_SAVE_FLAG));
		ESP_ERROR_CHECK(nvs_set_str(nvs_my_wifi_info_handler, "wifi_ssid", (const char *) ssid));
		ESP_ERROR_CHECK(nvs_set_str(nvs_my_wifi_info_handler, "wifi_passwd", (const char *) password));
		ESP_ERROR_CHECK(nvs_commit(nvs_my_wifi_info_handler)); /* 提交 */
		nvs_close(nvs_my_wifi_info_handler);                   /* 关闭 */
		ESP_LOGI(TAG_WIFI, "smartconfig save wifi info to NVS .");
		ESP_ERROR_CHECK(esp_wifi_disconnect());
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_connect());
	} else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
		xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
	}
}

void wifi_init(void)
{
	/* 定义一个NVS操作句柄 */
	nvs_handle nvs_my_wifi_info_handler;
	uint32_t   my_wifi_save_flag = 0;                // nvs 存储标志
	ESP_ERROR_CHECK(esp_netif_init());               // 初始化协议栈基于TCP/IP
	s_wifi_event_group = xEventGroupCreate();        // 创建事件组
	ESP_ERROR_CHECK(esp_event_loop_create_default());// 创建默认的事件组循环

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
	if (MY_WIFI_SAVE_FLAG == my_wifi_save_flag) {
		wifi_config_t wifi_config;
		char         *ssid          = malloc(sizeof(wifi_config.sta.ssid));
		char         *password      = malloc(sizeof(wifi_config.sta.password));
		size_t        required_size = 0;

		ESP_LOGI(TAG_WIFI, "wifi info is Already exists,direct networking.");
		required_size = sizeof(wifi_config.sta.ssid); /* 从NVS中获取ssid */
		ESP_ERROR_CHECK(nvs_get_str(nvs_my_wifi_info_handler, "wifi_ssid", ssid, &required_size));
		required_size = sizeof(wifi_config.sta.password); /* 从NVS中获取ssid */
		ESP_ERROR_CHECK(nvs_get_str(nvs_my_wifi_info_handler, "wifi_passwd", password, &required_size));
		ESP_ERROR_CHECK(nvs_commit(nvs_my_wifi_info_handler)); /* 提交 */
		bzero(&wifi_config, sizeof(wifi_config_t));


		memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
		memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
		ESP_LOGI(TAG_WIFI, "IN NVS SSID:%s", ssid);
		ESP_LOGI(TAG_WIFI, "IN NVS PASSWORD:%s", password);
		free(ssid);
		free(password);
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONFIGURED_BIT);
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_start());
		ESP_LOGI(TAG_WIFI, "wifi info in nvs is exists,connect directlly.");
	} else {
		xEventGroupSetBits(s_wifi_event_group, WIFI_NOT_CONFIGURED_BIT);
		ESP_ERROR_CHECK(esp_wifi_start());
		ESP_LOGI(TAG_WIFI, "wifi info in nvs is not exists,start smartconfig.");
	}
	nvs_close(nvs_my_wifi_info_handler); /* 关闭 */
}

void smartconfig_task(void *args)
{
	EventBits_t uxBits;
	ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));
	smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
	while (1) {
		uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
		if (uxBits & CONNECTED_BIT) {
			ESP_LOGI(TAG_WIFI, "WiFi Connected to ap");
			wifi_connected = true;
		}
		if (uxBits & ESPTOUCH_DONE_BIT) {
			ESP_LOGI(TAG_WIFI, "smartconfig over");
			esp_smartconfig_stop();
			vTaskDelete(NULL);
		}
	}
}
