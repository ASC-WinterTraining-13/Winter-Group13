#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include "zf_common_typedef.h"

#include "PID.h"
// pid结构体，位于"PID.h"（只是体现结构，不要解除注释）
//typedef struct {
//	float Target;
//	float Actual;
//	float Out;
//	
//	float Kp;
//	float Ki;
//	float Kd;
//	
//	float Error0;
//	float Error1;
//	float ErrorInt;
//	
//	float OutMax;
//	float OutMin;
//} PID_t;

//定义PID
//角度环
extern PID_t Angle_PID;

// 启停标志位
extern uint8_t Run_Flag;
//PWM输出
extern int16_t LeftPWM;		//左轮PWM
extern int16_t RightPWM;		//右轮PWM
extern int16_t AvePWM;			//平均PWM
extern int16_t DifPWM;			//差分PWM

// 调控周期计时器
extern uint16_t Time_Count1;

#endif