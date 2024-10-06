#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>

#define TXD_PIN 18   // RS485 TX pin
#define RXD_PIN 17    // RS485 RX pin //跟微雪官方文档相反
#define UART_PORT_NUM 1
#define BUF_SIZE (1024)

// CRC16校验计算函数
unsigned int calculateCRC16(const uint8_t *data, uint8_t length) {
    unsigned int crc = 0xFFFF;
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
void sendModbusCommand(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value) {
    uint8_t command[8];
    command[0] = address;  // 设备地址
    command[1] = functionCode;  // 功能码
    command[2] = registerAddress >> 8;  // 寄存器高字节
    command[3] = registerAddress & 0xFF;  // 寄存器低字节
    command[4] = value >> 8;  // 数据高字节
    command[5] = value & 0xFF;  // 数据低字节

    unsigned int crc = calculateCRC16(command, 6);  // 计算CRC
    command[6] = crc & 0xFF;  // CRC低字节
    command[7] = crc >> 8;  // CRC高字节

    uart_write_bytes(UART_PORT_NUM, (const char *)command, sizeof(command));  // 发送命令
    uart_wait_tx_done(UART_PORT_NUM, 100 / portTICK_PERIOD_MS);

}

// 读取响应数据
bool readModbusResponse(uint8_t *buffer, uint8_t expectedLength, int timeout_ms) {
    int len = uart_read_bytes(UART_PORT_NUM, buffer, expectedLength, pdMS_TO_TICKS(timeout_ms));
    return len == expectedLength;
}

// UART初始化
void init_rs485() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
}

void modbus_task(void *arg) {
     // 用于存储响应数据

    while (1) {
        // 光照传感器 0x01
        sendModbusCommand(0x01, 0x03, 0x0002, 0x0002);  // 读取光照寄存器
        uint8_t readResponse_1[9]; 
        if (readModbusResponse(readResponse_1, 9, 1000)) {
            uint16_t lightHigh = (readResponse_1[3] << 8) | readResponse_1[4];
            uint16_t lightLow  = (readResponse_1[5] << 8) | readResponse_1[6];
            uint32_t lightFull = ((uint32_t)lightHigh << 16) | lightLow;
            float lightValue = lightFull / 1000.0;
            ESP_LOGI("Modbus", "地址 0x01 光照强度: %.3f Lux", lightValue);
        } else {
            ESP_LOGW("Modbus", "读取地址 0x01 光照数据失败或无响应。");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 环境湿度和温度传感器 0x09
        sendModbusCommand(0x09, 0x03, 0x0000, 0x0002);  // 读取湿度和温度寄存器
        uint8_t readResponse_2[9]; 
        if (readModbusResponse(readResponse_2, 9, 1000)) {
            uint16_t moisture = (readResponse_2[3] << 8) | readResponse_2[4];
            uint16_t temperature = (readResponse_2[5] << 8) | readResponse_2[6];
            float moisture_float = moisture / 10.0;
            float temperature_float = temperature / 10.0;
            ESP_LOGI("Modbus", "地址 0x09 环境湿度: %.1f%% 环境温度: %.1f°C", moisture_float, temperature_float);
        } else {
            ESP_LOGW("Modbus", "读取地址 0x09 环境湿度/温度数据失败或无响应。");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 土壤传感器 0x03
        sendModbusCommand(0x03, 0x03, 0x0000, 0x0004);  // 读取土壤寄存器
        uint8_t readResponse_3[13]; 
        if (readModbusResponse(readResponse_3, 13, 1000)) {
            uint16_t water = (readResponse_3[3] << 8) | readResponse_3[4];
            uint16_t temp  = (readResponse_3[5] << 8) | readResponse_3[6];
            uint16_t EC    = (readResponse_3[7] << 8) | readResponse_3[8];
            uint16_t PH    = (readResponse_3[9] << 8) | readResponse_3[10];
            float water_float = water / 10.0;
            float temp_float  = temp / 10.0;
            float EC_float    = EC;
            float PH_float    = PH / 10.0;
            ESP_LOGI("Modbus", "地址 0x03 土壤湿度: %.1f%% 土壤温度: %.1f°C 土壤电导率: %.1f μS/cm 土壤PH: %.1f", water_float, temp_float, EC_float, PH_float);
        } else {
            ESP_LOGW("Modbus", "读取地址 0x03 土壤数据失败或无响应。");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    // 初始化RS485 UART
    init_rs485();

    // 创建Modbus任务
    xTaskCreate(modbus_task, "modbus_task", 8192, NULL, 10, NULL);

}
