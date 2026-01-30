#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include "zf_common_typedef.h"

#include "PID.h"
// 注：pid结构体，位于"PID.h"

//定义PID
extern PID_t Angle_PID;					//角度环
extern PID_t Speed_PID;					//速度环
extern PID_t Turn_PID;
extern PID_t TEMP_888_FUNC_4_PID;
extern PID_t TEMP_888_FUNC_5_PID;

//步长值
extern const float PID_STEPS[5][3];

// 启停标志位
extern uint8_t Run_Flag;
// MPU6050 分析使能标志
extern volatile uint8_t mpu6050_analysis_enable;
//// (由于蓝牙模块的使用)显示更新标志位
//extern uint8_t oled_Refresh;
// PWM输出
extern int16_t LeftPWM;					//左轮PWM
extern int16_t RightPWM;				//右轮PWM
extern int16_t AvePWM;					//平均PWM
extern int16_t DifPWM;					//差分PWM
extern int16_t LeftSpeed, RightSpeed;		//左轮实际速度，右轮实际速度
extern float AveSpeed, DifSpeed;		//平均实际速度，差分实际速度

// 调控周期计时器
extern uint16_t Time_Count1;
extern uint16_t Time_Count2;

#endif