#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "driver/gpio.h"
#include "driver/uart.h"
#include "main.h"

/* Sensor 数据结构 */
typedef struct {
	float luminance;   // 光照强度，单位 Lux
	float air_humidity;// 环境湿度，单位 %
	float air_temp;    // 环境温度，单位 °C
	float soil_water;  // 土壤湿度，单位 %
	float soil_temp;   // 土壤温度，单位 °C
	float soil_ec;     // 土壤电导率，单位 μS/cm
	float soil_ph;     // 土壤 pH 值

	float N;   // 氮浓度，单位 %
	float P;     // P浓度，单位 %
	float K;     // K浓度，单位 %
} sensor_data_t;

extern sensor_data_t sensor_data; // 声明全局变量

// 初始化接收使能引脚
void receive_enable_pin_init(void);

// CRC16校验计算函数
uint32_t calculate_crc16(const uint8_t *data, uint8_t length);

// 发送Modbus命令
void send_modbus_command(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value);

// 读取响应数据
bool read_modbus_response(uint8_t *buffer, uint8_t expectedLength, int timeout_ms);

// UART初始化
void rs485_init(void);

// Modbus任务
void modbus_task(void *arg);


#endif // __MODBUS_H__
