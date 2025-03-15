#ifndef __SOLENOID_H__
#define __SOLENOID_H__

#include "main.h"
#include "driver/gpio.h"

// 电磁阀相关操作
void solenoid_valves_init(void);
void solenoid_valves_open(void);
void solenoid_valves_close(void);

// 测试 LED 初始化
void led38_init(void);

#endif // __SOLENOID_H__
