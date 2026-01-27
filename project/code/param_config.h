#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include "zf_common_typedef.h"


#include "PID.h"
//pid结构体，位于"PID.h"（只是体现结构，不要解除注释）
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

//启停标志位
extern uint8_t Run_Flag;
//调控周期计时器
extern uint16_t Time_Count1;

#endif