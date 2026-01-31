#include "zf_device_oled.h"
#include "zf_device_mpu6050.h"

#include "param_config.h"
#include "param_storage.h"
#include "mpu6050_Analysis.h"
#include "motor.h"
#include "Encoder.h"
#include "bluetooth_ch04_example.h"
#include "zf_device_bluetooth_ch04.h"
#include "PID.h"

#include "math.h"

// PID计算参数重置
void PID_Init(PID_t *p)
{
	p->Target   = 0;
	p->Actual   = 0;
	p->Actual1  = 0;
	p->Out      = 0;
	p->Error0   = 0;
	p->Error1   = 0;
	p->ErrorInt = 0;	
}

//PID计算及结构体变量值更新
void PID_Update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->Error1 = p->Error0;					//获取上次误差
	p->Error0 = p->Target - p->Actual;		//获取本次误差，目标值减实际值，即为误差值
	
	float C = 1.0f;
	/*积分分离*/
	if ( fabs(p->Error0) > p->IntSepThresh )
	{
		C = 0.0f;
	}
	
	/*外环误差积分（累加）*/
	/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->Ki != 0)					//如果Ki不为0
	{
		p->ErrorInt += p->Error0;	//进行误差积分
		// 积分限幅
		if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
		if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;	
	}
	else							//否则
	{
		p->ErrorInt = 0;			//误差积分直接归0
	}
	
	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
//		   - p->Kd * (p->Actual - p->Actual1);//微分先行
	
	// 输出偏移
	if (p->Out > 0) {p->Out +=  p->OutOffset;}
	if (p->Out < 0) {p->Out -=  p->OutOffset;}
	
	//输出限幅
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin
	
//	p->Actual1 = p->Actual;//微分先行使用
}

// 平衡车平衡PID算法
void Balance_PID_Contorl(void)
{
	// 失控保护（加上调用层共两次，作为保险）
	if (Angle_Result < - 50 || 50 < Angle_Result)
	{
		motor_SetPWM(1, 0);
		motor_SetPWM(2, 0);
		
		return;
	}
	
	// 角度环PID计算
	Angle_PID.Actual = Angle_Result;
	PID_Update(&Angle_PID);
	
	// 编码器获取值

	
	// 换算
	AveSpeed = (LeftSpeed + RightSpeed) / 2.0;		//平均速度
	DifSpeed = LeftSpeed - RightSpeed;				//差分速度
	
	Speed_PID.Actual = AveSpeed;
	PID_Update(&Speed_PID);	
	
	AvePWM = - Angle_PID.Out + Speed_PID.Out; 
	
	//转向环
	Turn_PID.Actual = DifSpeed;
	PID_Update(&Turn_PID);
	DifPWM = Turn_PID.Out;
	
	// 输出换算
	LeftPWM  = AvePWM + DifPWM / 2;
	RightPWM = AvePWM - DifPWM / 2;						
	
	// 输出限幅
	if (LeftPWM  > 10000){LeftPWM = 10000;} else if (LeftPWM < -10000){LeftPWM = -10000;}
	if (RightPWM > 10000){RightPWM = 10000;}else if (RightPWM < -10000){RightPWM = -10000;}
	
	// 设置PWM
	motor_SetPWM(1, LeftPWM);
	motor_SetPWM(2, RightPWM);
	
}
