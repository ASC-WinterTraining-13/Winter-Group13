#include "param_config.h"

// 定义全局变量（真正分配内存的地方）

// 启停标志位
uint8_t Run_Flag = 0;
//PWM输出
int16_t LeftPWM = 0;		//左轮PWM
int16_t RightPWM = 0;		//右轮PWM
int16_t	AvePWM = 0;			//平均PWM
int16_t DifPWM = 0;			//差分PWM

// 调控周期计时器
uint16_t Time_Count1 = 0;

//定义PID
//角度环
PID_t Angle_PID = {
	.OutMax = 10000,
	.OutMin = -10000,
};
