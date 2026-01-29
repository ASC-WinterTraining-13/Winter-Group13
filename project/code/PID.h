#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;		//目标
	float Actual;		//实际
	float Out;			//输出
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt; 	//误差累积（积分）
	
	float OutMax;		//输出最大值
	float OutMin;		//输出最小值
	
	float IntSepThresh;  //积分分离阈值（误差大于阈值 不积分/积分清零）
} PID_t;

void PID_Update(PID_t *p);
void PID_Init(PID_t *p);

#endif
