#ifndef __PID_H
#define __PID_H

#include "zf_common_typedef.h"

typedef struct {
	float Target;			// 目标
	float Actual;			// 这次实际
	float Actual1;			// 上次实际（微分先行使用）
	float Out;				// 输出
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;			// 上次误差
	float Error1;			// 这次误差
	float ErrorInt; 		// 误差累积（积分）
	
	float ErrorIntMax;		// 积分最小值
	float ErrorIntMin;		// 积分最大值
	
	float OutMax;			// 输出最大值
	float OutMin;			// 输出最小值
	
	float OutOffset;		// 输出偏移
	float IntSepThresh;  	// 积分分离阈值（误差大于阈值 积分清零）
	
} PID_t;

void 	PID_Init				(PID_t *p);
void 	All_PID_Init			(void);
void 	PID_Update				(PID_t *p);
void 	PID_Calc_Speed_And_Turn	(void);
void 	PID_Calc_Angle_And_Rate	(void);
void 	Balance_PID_Contorl		(void);

#endif
