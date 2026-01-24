#include "zf_driver_gpio.h"

//代码思路基于江协"定时器实现非阻塞式程序"

#define KEY1                    (E2 )
#define KEY2                    (E3 )
#define KEY3                    (E4 )
#define KEY4                    (E5 )

uint8_t Key_Num;

//按键初始化
void Key_Init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY4 输入 默认高电平 上拉输入
	
}

//外部调用，查看被按下的按键
uint8_t Key_GetNum(void)
{
	uint8_t Temp;
//	if(Key_Num)//防止可能由中断造成的标志位直接清零，致使按键事件被忽略
//	{
		Temp = Key_Num;//利用中间变量实现标志位的返回并清零
		Key_Num = 0;
		return Temp;
//	}
//	return 0;
}

uint8_t Key_GetState(void)//获取当前按键状态的子函数
{
	if (!gpio_get_level(KEY1))//检测KEY1按下
	{
		return 1;
	}
	if (!gpio_get_level(KEY2))//检测KEY2按下
	{
		return 2;
	}
	if (!gpio_get_level(KEY3))//检测KEY3按下
	{
		return 3;
	}
	if (!gpio_get_level(KEY4))//检测KEY4按下
	{
		return 4;
	}
	return 0;//没有任何按键按下
}

uint8_t Key_Tick(void)
{
	static uint8_t CurrState,PrevState;//Current,Previous
	//静态变量默认值为0，函数退出后值不会丢失
	
	PrevState = CurrState;
	CurrState = Key_GetState();
	
	if(CurrState == 0 && PrevState != 0)//捕获按键松手瞬间
	{
		Key_Num = PrevState;//键码标志位
	}
}




