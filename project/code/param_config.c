#include "param_config.h"

// 定义全局变量（真正分配内存的地方）

// 启停标志位
uint8_t Run_Flag = 0;
// MPU6050 分析使能标志
volatile uint8_t mpu6050_analysis_enable = 0;
//// (由于蓝牙模块的使用)显示更新标志位
//uint8_t oled_Refresh = 0;
//PWM输出
int16_t LeftPWM  = 0;		//左轮PWM
int16_t RightPWM = 0;		//右轮PWM
int16_t	AvePWM   = 0;			//平均PWM
int16_t DifPWM   = 0;			//差分PWM

float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

// 调控周期计时器
uint16_t Time_Count1 = 0;
uint16_t Time_Count2 = 0;

//定义PID
//角度环
PID_t Angle_PID = {
	.OutMax = 9000,
	.OutMin = -9000,
	.IntSepThresh = 6,		//积分分离阈值
	.OutOffset = 0,			//输出偏移
};
//速度环
PID_t Speed_PID = {	
	.OutMax = 20,
	.OutMin = -20,
	.IntSepThresh = 20,		//积分分离阈值
	.OutOffset = 0,			//输出偏移
};
//转向环
PID_t Turn_PID = {	
	.OutMax = 5000,
	.OutMin = -5000,
	.IntSepThresh = 5000,	//积分分离阈值
	.OutOffset = 0,			//输出偏移
};

PID_t TEMP_888_FUNC_4_PID = {	
	
};

PID_t TEMP_888_FUNC_5_PID = {	
	
};

//pid更改步长值
const float PID_STEPS[5][3] = {
    {1.0f, 0.1f, 0.1f},   	// Angle_PID：Kp=1.0, Ki=0.01, Kd=0.1
    {0.5f, 0.02f, 0.05f}, 	// Speed_PID：Kp=0.5, Ki=0.02, Kd=0.05
    {0.2f, 0.005f, 0.2f},  	// ?????_PID：Kp=0.2, Ki=0.005, Kd=0.2
    {0.1f, 0.01f, 0.1f},  	// ?????_PID：Kp=0.8, Ki=0.015, Kd=0.1
    {0.1f, 0.01f, 0.1f}   	// ?????_PID：Kp=0.3, Ki=0.01, Kd=0.15
};
