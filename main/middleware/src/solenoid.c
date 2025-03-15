#include "solenoid.h"

// 电磁阀
void solenoid_valves_init(void)
{
	gpio_config_t io_config;
	io_config.mode         = GPIO_MODE_OUTPUT;
	io_config.pull_up_en   = GPIO_PULLUP_DISABLE;
	io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_config.pin_bit_mask = 1ULL << SOLENOID_VALVES_PIN;
	io_config.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&io_config);
}

// 测试
void led38_init(void)
{
	gpio_config_t io_config;
	io_config.mode         = GPIO_MODE_OUTPUT;
	io_config.pull_up_en   = GPIO_PULLUP_DISABLE;
	io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_config.pin_bit_mask = 1ULL << GPIO_NUM_38;
	io_config.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&io_config);
}

void solenoid_valves_open(void)
{
	gpio_set_level(SOLENOID_VALVES_PIN, 1);
	gpio_set_level(SOLENOID_LED_PIN, 1);
}

void solenoid_valves_close(void)
{
	gpio_set_level(SOLENOID_VALVES_PIN, 0);
	gpio_set_level(SOLENOID_LED_PIN, 0);
}