#include "modbus.h"
#include "solenoid.h"
#include "wifi.h"
#include "main.h"
#include "mqtt_cb.h"
#include "mqtt_thread.h"
#include "json.h"
#include "OTA.h"
#include "report.h"
#include "send.h"
#include "sdkconfig.h"

//调试用
const char *TAG_WIFI = "WIFI";         // 在主文件中定义TAG_WIFI
const char *TAG_OTA = "OTA";         // 在主文件中定义TAG_OTA
static const char *TAG_MAIN         = "MAIN";                 // LOG 标签

//sdkconfig中获取
char              *cur_version = CONFIG_FIRMWARE_VERSION;// 固件更新时将此版本号更改为跟阿里云上面设置的一样的格式即可例如：“1.0.0”、“1.0.1”
const char *product_key   = CONFIG_PRODUCT_KEY;//product_key
const char *device_name   = CONFIG_DEVICE_NAME;//device_name
const char *device_secret = CONFIG_DEVICE_SECRET;//device_secret

//变量
sensor_data_t sensor_data;// 创建一个 SensorData 类型的全局变量或静态变量
bool water_switch = false;

//外部的证书
/* 位于portfiles/aiot_port文件夹下的系统适配函数集合 */
extern aiot_sysdep_portfile_t g_aiot_sysdep_portfile;
/* 位于external/ali_ca_cert.c中的服务器证书 */
extern const char *ali_ca_cert;

//OTA句柄
void  *g_ota_handle    = NULL;
void  *ota_handle      = NULL;
void    *mqtt_handle = NULL;
void *dm_handle = NULL;
//uint32_t           g_firmware_size = 0;

//线程
static pthread_t g_mqtt_process_thread;
static pthread_t g_mqtt_recv_thread;
static pthread_t ota_thread;

//标志位
uint8_t g_mqtt_process_thread_running = 0;
uint8_t g_mqtt_recv_thread_running    = 0;
uint8_t ota_running                   = 0;//没有用到的标志位

void app_main(void)
{
	// 硬件初始化
	hardware_init();

	//WIFI连接
	wifi_start();

	// loop
	//ESP_LOGI(TAG_MAIN, "Start alink main");
	alink_main();
}

int mqtt_config_init(void)
{
	/*************
     * mqtt服务器初始化
     * ***************/

	char    *url         = CONFIG_ALICLOUD_URL; /* 阿里云平台上海站点的域名后缀 */
	char     host[100]   = {0};  /* 用这个数组拼接设备连接的云平台站点全地址, 规则是 ${productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com */
	uint16_t port        = 1883; /* 无论设备是否使用TLS连接阿里云平台, 目的端口都是443 */
	aiot_sysdep_network_cred_t cred; /* 安全凭据结构体, 如果要用TLS, 这个结构体中配置CA证书等参数 */
	uint8_t post_reply = 1;

	// 设备管理模块的消息接收处理器为 demo_dm_recv_handler
	aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *) demo_dm_recv_handler);
	// 设置了是否需要云端回复设备发送的POST请求
	aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *) &post_reply);

	/* 配置SDK的底层依赖 */
	aiot_sysdep_set_portfile(&g_aiot_sysdep_portfile);
	/* 配置SDK的日志输出 */
	aiot_state_set_logcb(demo_state_logcb);

	/* 创建SDK的安全凭据, 用于建立TLS连接 */
	memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
	cred.option               = AIOT_SYSDEP_NETWORK_CRED_SVRCERT_CA; /* 使用RSA证书校验MQTT服务端 */
	cred.max_tls_fragment     = 16384;                               /* 最大的分片长度为16K, 其它可选值还有4K, 2K, 1K, 0.5K */
	cred.sni_enabled          = 1;                                   /* TLS建连时, 支持Server Name Indicator */
	cred.x509_server_cert     = ali_ca_cert;                         /* 用来验证MQTT服务端的RSA根证书 */
	cred.x509_server_cert_len = strlen(ali_ca_cert);                 /* 用来验证MQTT服务端的RSA根证书长度 */

	/* 创建1个MQTT客户端实例并内部初始化默认参数 */
	mqtt_handle = aiot_mqtt_init();
	if (mqtt_handle == NULL) {
		printf("aiot_mqtt_init failed\n");
		return -1;
	}

	/* TODO: 如果以下代码不被注释, 则例程会用TCP而不是TLS连接云平台 */

	{
		memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
		cred.option = AIOT_SYSDEP_NETWORK_CRED_NONE;
	}

	// snprintf(host, 100, "%s.%s", product_key, url);
	ESP_LOGI(TAG_MAIN, "url: %s", url);  // 打印 url

	snprintf(host, 100, "%s", url);
	ESP_LOGI(TAG_MAIN, "MQTT host: %s", host);

	/* 配置MQTT服务器地址 */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_HOST, (void *) host);
	/* 配置MQTT服务器端口 */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PORT, (void *) &port);
	/* 配置设备productKey */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PRODUCT_KEY, (void *) product_key);
	/* 配置设备deviceName */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_NAME, (void *) device_name);
	/* 配置设备deviceSecret */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_SECRET, (void *) device_secret);
	/* 配置网络连接的安全凭据, 上面已经创建好了 */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_NETWORK_CRED, (void *) &cred);
	/* 配置MQTT默认消息接收回调函数 */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_RECV_HANDLER, (void *) demo_mqtt_default_recv_handler);
	/* 配置MQTT事件回调函数 */
	aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_EVENT_HANDLER, (void *) demo_mqtt_event_handler);

	/* 与MQTT例程不同的是, 这里需要增加创建OTA会话实例的语句 */
	ota_handle = aiot_ota_init();
	if (NULL == ota_handle) {
		return -1;
	}

	/* 用以下语句, 把OTA会话和MQTT会话关联起来 */
	aiot_ota_setopt(ota_handle, AIOT_OTAOPT_MQTT_HANDLE, mqtt_handle);
	/* 用以下语句, 设置OTA会话的数据接收回调, SDK收到OTA相关推送时, 会进入这个回调函数 */
	aiot_ota_setopt(ota_handle, AIOT_OTAOPT_RECV_HANDLER, user_ota_recv_handler);
	g_ota_handle = ota_handle;

	/**创建MQTT**/
	/* 创建DATA-MODEL实例 */
	dm_handle = aiot_dm_init();
	if (dm_handle == NULL) {
		printf("aiot_dm_init failed");
		return -1;
	}
	/* 配置MQTT实例句柄 */
	aiot_dm_setopt(dm_handle, AIOT_DMOPT_MQTT_HANDLE, mqtt_handle);
	/* 配置消息接收处理回调函数 */
	aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *) demo_dm_recv_handler);

	/* 配置是云端否需要回复post_reply给设备. 如果为1, 表示需要云端回复, 否则表示不回复 */
	aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *) &post_reply);

	return  0;
}

int alink_main(void)
{
	mqtt_config_init();
	/*****************************
     * 客户端，设备初始化
     *****************************/
	int32_t  res         = STATE_SUCCESS;
	/* 与服务器建立MQTT连接 */
	res = aiot_mqtt_connect(mqtt_handle);
	if (res < STATE_SUCCESS) {
		/* 尝试建立连接失败, 销毁MQTT实例, 回收资源 */
		aiot_mqtt_deinit(&mqtt_handle);
		printf("aiot_mqtt_connect failed: -0x%04lX\n", -res);
		goto exit;
	}

	/* 演示MQTT连接建立起来之后, 就可以上报当前设备的版本号了 */
	res = aiot_ota_report_version(ota_handle, cur_version);
	if (res < STATE_SUCCESS) {
		printf("report version failed, code is -0x%04lX\r\n", -res);
	}

	/* MQTT 订阅topic功能示例, 请根据自己的业务需求进行使用 */
	// 设备上报属性之后，云端对设备的回复，通过这种方式来保证数据更新到云端
	{
		char *sub_topic = MQTT_SUB_TOPIC_PROPERTY;

		res = aiot_mqtt_sub(mqtt_handle, sub_topic, NULL, 1, NULL);
		if (res < 0) {
			printf("aiot_mqtt_sub failed, res: -0x%04lX\n", -res);
			return -1;
		}
	}

	/* MQTT 发布消息功能示例, 请根据自己的业务需求进行使用 */
	// 云端给设备发送指令，如控制开关等
	{
		char *pub_topic   = MQTT_PUB_TOPIC_PROPERTY;
		char *pub_payload = "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"LightSwitch\":0}}";

		res = aiot_mqtt_pub(mqtt_handle, pub_topic, (uint8_t *) pub_payload, strlen(pub_payload), 0);
		if (res < 0) {
			printf("aiot_mqtt_sub failed, res: -0x%04lX\n", -res);
			return -1;
		}
	}

	/* 创建一个单独的线程, 专用于执行aiot_mqtt_process, 它会自动发送心跳保活, 以及重发QoS1的未应答报文 */
	g_mqtt_process_thread_running = 1;
	res                           = pthread_create(&g_mqtt_process_thread, NULL, demo_mqtt_process_thread, mqtt_handle);
	if (res < 0) {
		printf("pthread_create demo_mqtt_process_thread failed: %ld\n", res);
		return -1;
	}

	/* 创建一个单独的线程用于执行aiot_mqtt_recv, 它会循环收取服务器下发的MQTT消息, 并在断线时自动重连 */
	g_mqtt_recv_thread_running = 1;
	res                        = pthread_create(&g_mqtt_recv_thread, NULL, demo_mqtt_recv_thread, mqtt_handle);
	if (res < 0) {
		printf("pthread_create demo_mqtt_recv_thread failed: %ld\n", res);
		return -1;
	}

	ota_running = 1;//没有用到的标志位
	res         = pthread_create(&ota_thread, NULL, ota_task, NULL);
	if (res < 0) {
		printf("pthread_create ota_task failed: %ld\n", res);
		return -1;
	}

	/* 主循环进入休眠 */
	while (1) {
		char                   buf[128];
		json_gen_str_t         jstr;
		json_gen_test_result_t result;
		memset(&result, 0, sizeof(json_gen_test_result_t));

		json_gen_str_start(&jstr, buf, sizeof(buf), flush_str, &result);
		json_gen_start_object(&jstr);

		json_gen_obj_set_bool(&jstr, "WaterOutletSwitch", water_switch);
		json_gen_obj_set_float(&jstr, "CurrentTemperature", sensor_data.air_temp);
		json_gen_obj_set_float(&jstr, "RelativeHumidity", sensor_data.air_humidity);
		json_gen_obj_set_float(&jstr, "LightLux", sensor_data.luminance);
		json_gen_obj_set_float(&jstr, "SoilTemperature", sensor_data.soil_temp);
		json_gen_obj_set_float(&jstr, "SoilHumidity", sensor_data.soil_water);
		json_gen_obj_set_float(&jstr, "SoilPH", sensor_data.soil_ph);
		json_gen_obj_set_float(&jstr, "SoilEC", sensor_data.soil_ec);
		json_gen_obj_set_float(&jstr, "SoilN", sensor_data.N);
		json_gen_obj_set_float(&jstr, "SoilP", sensor_data.P);
		json_gen_obj_set_float(&jstr, "SoilK", sensor_data.K);

		json_gen_end_object(&jstr);
		json_gen_str_end(&jstr);

		demo_send_property_post(dm_handle, result.buf);

		//		TODO: 上报周期
		vTaskDelay(pdMS_TO_TICKS(PUB_TIME * 1000));
	}

exit:
	/* 停止收发动作 */
	g_mqtt_process_thread_running = 0;
	g_mqtt_recv_thread_running    = 0;

	// 断开MQTT连接, 一般不会运行到这里
	res = aiot_mqtt_disconnect(mqtt_handle);
	if (res < STATE_SUCCESS) {
		aiot_mqtt_deinit(&mqtt_handle);
		printf("aiot_mqtt_disconnect failed: -0x%04lX\n", -res);
		return -1;
	}

	//* 销毁MQTT实例, 一般不会运行到这里
	res = aiot_mqtt_deinit(&mqtt_handle);
	if (res < STATE_SUCCESS) {
		printf("aiot_mqtt_deinit failed: -0x%04lX\n", -res);
		return -1;
	}

	/* 销毁OTA实例, 一般不会运行到这里 */
	aiot_ota_deinit(&ota_handle);

	g_mqtt_process_thread_running = 0;
	g_mqtt_recv_thread_running    = 0;
	pthread_join(g_mqtt_process_thread, NULL);
	pthread_join(g_mqtt_recv_thread, NULL);

	return 0;
}

void hardware_init(void)
{
	// 初始化RS485 UART
	rs485_init();
	// 初始化电磁阀
	solenoid_valves_init();
	// 初始化LED
	led38_init();

	// 创建Modbus任务
	xTaskCreate(modbus_task, "modbus_task", 8192, NULL, 10, NULL);
	//ESP_LOGI(TAG_MAIN,"硬件初始化完成\r\n");
}

void wifi_start(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());  // 初始化 NVS
	wifi_init();       // wifi初始化函数
	while (!wifi_connected) {
		printf("no wifi\r\n");
		vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_TIME * 1000));
	}
	//ESP_LOGI(TAG_MAIN,"WIFI已连接\r\n");
}