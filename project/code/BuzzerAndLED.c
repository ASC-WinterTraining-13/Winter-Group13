#include "zf_driver_gpio.h"

#define BEEP		D7
#define LED2		B13

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     声光模块初始化
// 使用示例     BuzzerAndLED_Init();
// 备注信息     只在"Menu.c"中的Peripheral_Init();函数调用一次
//-------------------------------------------------------------------------------------------------------------------

void BuzzerAndLED_Init(void)
{
	gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);	
	gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);	
}  

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开关蜂鸣器/LED
// 使用示例     BuzzerAndLED_Promopt(1);
// 备注信息     只在"Menu.c"中的Peripheral_Init();函数调用一次
//-------------------------------------------------------------------------------------------------------------------

void BuzzerAndLED_Promopt(uint8_t Mode)
{
	if (Mode == 1) gpio_set_level (BEEP, GPIO_HIGH);	// 蜂鸣器开
	if (Mode == 2) gpio_set_level (LED2, GPIO_LOW);		// LED开
	if (Mode == 3) gpio_set_level (BEEP, GPIO_LOW);		// 蜂鸣器关
	if (Mode == 4) gpio_set_level (LED2, GPIO_HIGH);	// LED关
}
