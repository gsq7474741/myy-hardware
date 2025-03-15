/*****************************************OTA********************************************* */

#include "OTA.h"

esp_err_t err;
/* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
esp_ota_handle_t       update_handle    = 0;
const esp_partition_t *update_partition = NULL;

const esp_partition_t *configured = NULL;
const esp_partition_t *running    = NULL;
void  *g_dl_handle = NULL;

static void __attribute__((noreturn)) task_fatal_error(void)
{
	ESP_LOGE(TAG_OTA, "Exiting task due to fatal error...");
	(void) vTaskDelete(NULL);

	while (1) {
		;
	}
}

/* 下载收包回调, 用户调用 aiot_download_recv() 后, SDK收到数据会进入这个函数, 把下载到的数据交给用户 */
void user_download_recv_handler(void *handle, const aiot_mqtt_download_recv_t *packet, void *userdata)
{
	uint32_t data_buffer_len = 0;

	/* 目前只支持packet->type为AIOT_MDRECV_DATA_RESP的情况 */
	if (!packet || AIOT_MDRECV_DATA_RESP != packet->type) {
		return;
	}

	/* 应在此实现文件本地固化的操作 */
	err = esp_ota_write(update_handle, (const void *) packet->data.data_resp.data, packet->data.data_resp.data_size);
	if (err != ESP_OK) {
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
	switch (ota_msg->type) {
		case AIOT_OTARECV_FOTA: {
			if (NULL == ota_msg->task_desc || ota_msg->task_desc->protocol_type != AIOT_OTA_PROTOCOL_MQTT) {
				break;
			}
			printf("OTA target firmware version: %s, size: %lu Bytes\r\n", ota_msg->task_desc->version, ota_msg->task_desc->size_total);

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

void *ota_task(void *args)
{
	ESP_LOGI(TAG_OTA, "Starting OTA example task");

	configured = esp_ota_get_boot_partition();
	running    = esp_ota_get_running_partition();

	if (configured != running) {
		ESP_LOGW(
				TAG_OTA,
				"Configured OTA boot partition at offset 0x%08" PRIx32 ", but running from offset 0x%08" PRIx32,
				configured->address,
				running->address);
		ESP_LOGW(TAG_OTA, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
	}
	ESP_LOGI(TAG_OTA, "Running partition type %d subtype %d (offset 0x%08" PRIx32 ")", running->type, running->subtype, running->address);

	// 获取目标分区
	update_partition = esp_ota_get_next_update_partition(NULL);
	assert(update_partition != NULL);
	ESP_LOGI(TAG_OTA, "Writing to partition subtype %d at offset 0x%" PRIx32, update_partition->subtype, update_partition->address);

	err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_OTA, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		esp_ota_abort(update_handle);
		task_fatal_error();
	}
	ESP_LOGI(TAG_OTA, "esp_ota_begin succeeded");

	while (1) {
		if (g_dl_handle != NULL) {
			int32_t res = aiot_mqtt_download_process(g_dl_handle);

			if (STATE_MQTT_DOWNLOAD_SUCCESS == res) {
				/* 升级成功，可在此处重启并且上报新的版本号 */
				printf("mqtt download ota success \r\n");
				err = esp_ota_end(update_handle);
				if (err != ESP_OK) {
					if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
						ESP_LOGE(TAG_OTA, "Image validation failed, image is corrupted");
					} else {
						ESP_LOGE(TAG_OTA, "esp_ota_end failed (%s)!", esp_err_to_name(err));
					}
					task_fatal_error();
				}
				err = esp_ota_set_boot_partition(update_partition);
				if (err != ESP_OK) {
					ESP_LOGE(TAG_OTA, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
					task_fatal_error();
				}
				ESP_LOGI(TAG_OTA, "Prepare to restart system!");
				aiot_mqtt_download_deinit(&g_dl_handle);
				esp_restart();
				return NULL;
				// aiot_mqtt_download_deinit(&g_dl_handle);
				// break;
			} else if (
					STATE_MQTT_DOWNLOAD_FAILED_RECVERROR == res || STATE_MQTT_DOWNLOAD_FAILED_TIMEOUT == res ||
					STATE_MQTT_DOWNLOAD_FAILED_MISMATCH == res) {
				printf("mqtt download ota failed \r\n");
				aiot_mqtt_download_deinit(&g_dl_handle);
				break;
			}
		}
		sleep(1);
	}
	return NULL;
}