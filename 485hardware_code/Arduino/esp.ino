#include <HardwareSerial.h>
#define RS485Serial Serial2
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
 
    RS485Serial.write(command, 8);  // 发送命令
}
 
// 读取响应数据
int read_time2=1000;
bool readModbusResponse(uint8_t *buffer, uint8_t expectedLength) {
    uint32_t startTime = millis();
    uint8_t index = 0;
    while (millis() - startTime < read_time2) {  // 1秒超时
        if (RS485Serial.available()) {
            buffer[index++] = RS485Serial.read();
            if (index == expectedLength) {
                return true;
            }
        }
    }
    return false;
}
//-----------------------------检查传感器是否存在
#define MAX_ADDRESS 0x3F  // 最大地址 0x3F
bool validAddresses[MAX_ADDRESS + 1];  // 存储地址是否有效的数组
bool addressesChecked = false;  // 标志位，表示地址是否已检查完毕
void checkAddresses() {
  
  int read_time3=read_time2;
  read_time2=500;
    // for (uint8_t i = 0x01; i <= MAX_ADDRESS; i++) {
    for (uint8_t i = 0x01; i <= 0x02; i++) {
        sendModbusCommand(i, 0x03, 0x0000, 0x0004);  // 读取2个寄存器
        uint8_t readResponse[9];
        if (readModbusResponse(readResponse,9)) {
            validAddresses[i] = true;
            Serial.print("地址 0x");
            Serial.print(i, HEX);
            Serial.println(" 有效。");
        } else {
            validAddresses[i] = false;
            Serial.print("地址 0x");
            Serial.print(i, HEX);
            Serial.println(" 无响应或无效。");
        }
    }
    read_time2=read_time3;
    addressesChecked = true;  // 设置标志位，表示地址已检查完毕
}

void change(){
  if (Serial.available()) {
    int newAddress = Serial.parseInt();  // 获取用户输入的新地址
 
    // 检查输入地址是否有效
    if (newAddress >= 1 && newAddress <= 247) {
      uint8_t broadcastAddress = 0xFF;  // 广播地址
      
      // 创建Modbus命令帧
      uint8_t message[8];
      message[0] = broadcastAddress;  // 广播地址
      message[1] = 0x06;              // 功能码：写单个寄存器
      message[2] = 0x07;              // 寄存器地址高字节（0x07D0）
      message[3] = 0xD0;              // 寄存器地址低字节
      message[4] = 0x00;              // 新地址高字节
      message[5] = newAddress;        // 新地址低字节
 
      // 计算CRC
      uint16_t crc = calculateCRC16(message, 6);
      message[6] = crc & 0xFF;         // CRC低字节
      message[7] = (crc >> 8) & 0xFF;  // CRC高字节
 
      // 发送数据到传感器
      RS485Serial.write(message, 8);
 
      // 延迟等待传感器响应
      delay(100);
 
      // 提示用户地址更改已发送
      Serial.print("已发送地址更改命令，新的地址为：");
      Serial.println(newAddress);
    } else {
      Serial.println("无效的地址，请输入1到247之间的数字。");
    }
 
    // 继续等待下一个输入
    Serial.println("请输入新地址(1-247)：");
  }
}
void setup() {
    // 初始化调试串口
    Serial.begin(115200);
    // 初始化RS485串口
    RS485Serial.begin(9600, SERIAL_8N1, 17, 18);  // RX: GPIO18, TX: GPIO17
}
void loop() {
            
            
            
            sendModbusCommand(0x01, 0x03, 0x0002, 0x0002);  // 光照
            uint8_t readResponse_1[9];
            if (readModbusResponse(readResponse_1, 9)) {
                uint16_t lightHigh = (readResponse_1[3] << 8) | readResponse_1[4];
                uint16_t lightLow  = (readResponse_1[5] << 8) | readResponse_1[6];
                uint32_t lightFull = ((uint32_t)lightHigh << 16) | lightLow;
                // 将湿度和温度转换为浮点型
                float lightValue = lightFull / 1000.0;
                
                Serial.print("\n地址 0x01");
                Serial.print("\n光照强度: ");
                Serial.print(lightValue);
                Serial.println("Lux");

            } else {
                Serial.print("读取地址 0x01");
                Serial.println(" 光照数据失败或无响应。");
            }
            delay(1000);
    
            
            

            sendModbusCommand(0x09, 0x03, 0x0000, 0x0002);  // 读取2个寄存器
            uint8_t readResponse_2[9];
            if (readModbusResponse(readResponse_2, 9)) {
                uint16_t moisture = (readResponse_2[3] << 8) | readResponse_2[4];
                uint16_t temperature = (readResponse_2[5] << 8) | readResponse_2[6];
                // 将湿度和温度转换为浮点型
                float moisture_float = moisture / 10.0;
                float temperature_float = temperature / 10.0;
                Serial.print("\n地址 0x2");
                Serial.print("\n环境湿度: ");
                Serial.print(moisture_float);
                Serial.println("%");
                Serial.print("\n环境温度: ");
                Serial.println(temperature_float);
            } else {
                Serial.print("读取地址 0x2");
                Serial.println(" 环境温湿度数据失败或无响应。");
            }

            delay(1000);
            
            
            
            sendModbusCommand(0x03, 0x03, 0x0000, 0x0004);  // 土壤
            uint8_t readResponse_3[13];
            if (readModbusResponse(readResponse_3, 13)) {
                uint16_t water = (readResponse_3[3] << 8) | readResponse_3[4];
                uint16_t temp  = (readResponse_3[5] << 8) | readResponse_3[6];
                uint16_t  EC   = (readResponse_3[7] << 8) | readResponse_3[8];
                uint16_t  PH   = (readResponse_3[9] << 8) | readResponse_3[10];
                
                // 将湿度和温度转换为浮点型
                float water_float = water / 10.0;
                float temp_float  = temp  / 10.0;
                float EC_float  = EC;
                float PH_float  = PH  / 10.0;
                // if(moisture_float>101||temperature_float>200){
                //   checkAddresses();
                // }
                Serial.print("\n地址 0x03");
                Serial.print("\n土壤湿度:");
                Serial.print(water_float);
                Serial.println( "% ");
                Serial.print("\n土壤温度:");
                Serial.print(temp_float);
                Serial.println(" ℃ ");
                Serial.print("\n土壤电导率:");
                Serial.print(EC_float);
                Serial.println(" μS/cm ");
                Serial.print("\n土壤PH:");
                Serial.print(PH_float);

            } else {
                Serial.print("读取地址 0x03");
                Serial.println(" 土壤数据失败或无响应。");
            }
            delay(1000);
        
    
    delay(1000);  // 每隔一秒读取一次，防止过度读取
}