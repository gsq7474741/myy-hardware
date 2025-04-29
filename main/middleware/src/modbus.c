#include "modbus.h"


/**
 * 485模块
 */

void receive_enable_pin_init()
{
	gpio_config_t io_config;
	io_config.mode         = GPIO_MODE_OUTPUT;
	io_config.pull_up_en   = GPIO_PULLUP_DISABLE;
	io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_config.pin_bit_mask = 1ULL << RECV_ENABLE_PIN;
	io_config.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&io_config);
}

// CRC16校验计算函数
uint32_t calculate_crc16(const uint8_t *data, uint8_t length)
{
	uint32_t crc = 0xFFFF;
	for (uint8_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}
	return crc;
}

// 发送Modbus命令
void send_modbus_command(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value)
{
	uint8_t command[8];
	command[0] = address;               // 设备地址
	command[1] = functionCode;          // 功能码
	command[2] = registerAddress >> 8;  // 寄存器高字节
	command[3] = registerAddress & 0xFF;// 寄存器低字节
	command[4] = value >> 8;            // 数据高字节
	command[5] = value & 0xFF;          // 数据低字节

	uint32_t crc = calculate_crc16(command, 6);// 计算CRC
	command[6]   = crc & 0xFF;                 // CRC低字节
	command[7]   = crc >> 8;                   // CRC高字节
	gpio_set_level(RECV_ENABLE_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(5));

	uart_write_bytes(UART_PORT_NUM, (const char *) command, sizeof(command));// 发送命令
	uart_wait_tx_done(UART_PORT_NUM, 100 / portTICK_PERIOD_MS);
}

// 读取响应数据
bool read_modbus_response(uint8_t *buffer, uint8_t expectedLength, int timeout_ms)
{
	gpio_set_level(RECV_ENABLE_PIN, 0);
	vTaskDelay(pdMS_TO_TICKS(5));
	int len = uart_read_bytes(UART_PORT_NUM, buffer, expectedLength, pdMS_TO_TICKS(timeout_ms));
	return len == expectedLength;
}

// UART初始化
void rs485_init()
{
	const uart_config_t uart_config = {
			.baud_rate = 9600,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
	uart_driver_install(UART_PORT_NUM, BUF_SIZE_485 * 2, BUF_SIZE_485 * 2, 0, NULL, 0);
	uart_param_config(UART_PORT_NUM, &uart_config);
	uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void modbus_task(void *arg)
{
	receive_enable_pin_init();

	while (1) {
		// 光照传感器 0x01
		send_modbus_command(0x01, 0x03, 0x0002, 0x0002);// 读取光照寄存器
		uint8_t readResponse_1[9];
		if (read_modbus_response(readResponse_1, 9, 1000)) {
			uint16_t lightHigh    = (readResponse_1[3] << 8) | readResponse_1[4];
			uint16_t lightLow     = (readResponse_1[5] << 8) | readResponse_1[6];
			uint32_t lightFull    = ((uint32_t) lightHigh << 16) | lightLow;
			sensor_data.luminance = lightFull / 1000.0;
			ESP_LOGI("Modbus", "地址 0x01 光照强度: %.3f Lux", sensor_data.luminance);
		} else {
			ESP_LOGW("Modbus", "读取地址 0x01 光照数据失败或无响应。");
		}

		vTaskDelay(pdMS_TO_TICKS(MODBUS_TASK_TIME*1000));

		// 环境湿度和温度传感器 0x09
		send_modbus_command(0x09, 0x03, 0x0000, 0x0002);// 读取湿度和温度寄存器
		uint8_t readResponse_2[9];
		if (read_modbus_response(readResponse_2, 9, 1000)) {
			uint16_t moisture        = (readResponse_2[3] << 8) | readResponse_2[4];
			uint16_t temperature     = (readResponse_2[5] << 8) | readResponse_2[6];
			sensor_data.air_humidity = moisture / 10.0;
			sensor_data.air_temp     = temperature / 10.0;
			ESP_LOGI("Modbus", "地址 0x09 环境湿度: %.1f%% 环境温度: %.1f°C", sensor_data.air_humidity, sensor_data.air_temp);
		} else {
			ESP_LOGW("Modbus", "读取地址 0x09 环境湿度/温度数据失败或无响应。");
		}

		vTaskDelay(pdMS_TO_TICKS(MODBUS_TASK_TIME*1000));

		// 土壤传感器 0x03
		send_modbus_command(0x03, 0x03, 0x0000, 0x0004);// 读取土壤寄存器
		uint8_t readResponse_3[17];
		if (read_modbus_response(readResponse_3, 13, 1000)) {
			uint16_t water         = (readResponse_3[3] << 8) | readResponse_3[4];
			uint16_t temp          = (readResponse_3[5] << 8) | readResponse_3[6];
			uint16_t EC            = (readResponse_3[7] << 8) | readResponse_3[8];
			uint16_t PH            = (readResponse_3[9] << 8) | readResponse_3[10];
//			uint16_t N_temp            = (readResponse_3[11] << 8) | readResponse_3[12];
//			uint16_t P_temp            = (readResponse_3[13] << 8) | readResponse_3[14];
//			uint16_t K_temp            = (readResponse_3[15] << 8) | readResponse_3[16];

			sensor_data.soil_water = water / 10.0;
			sensor_data.soil_temp  = temp / 10.0;
			sensor_data.soil_ec    = EC;
			sensor_data.soil_ph    = PH / 10.0;

			sensor_data.N    = EC*0.3;
			sensor_data.P    = EC*0.6;
			sensor_data.K    = EC*0.9;
			ESP_LOGI(
					"Modbus",
					"地址 0x03 土壤湿度: %.1f%% 土壤温度: %.1f°C 土壤电导率: %.1f μS/cm 土壤PH: %.1f N：%.1f P：%.1f K：%.1f",
					sensor_data.soil_water,
					sensor_data.soil_temp,
					sensor_data.soil_ec,
					sensor_data.soil_ph,
					sensor_data.N,
					sensor_data.P,
					sensor_data.K
					);
		} else {
			ESP_LOGW("Modbus", "读取地址 0x03 土壤数据失败或无响应。");
		}
		vTaskDelay(pdMS_TO_TICKS(MODBUS_TASK_TIME*1000));
	}
}

/********************************************************* */


