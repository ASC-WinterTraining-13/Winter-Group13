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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID计算中间量重置
// 参数说明     PID_t *p	要传入的PID结构体变量名称
// 使用示例     PID_Init(&Angle_PID);
// 备注信息     注意需要考虑多个调用位置
//-------------------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID计算中间量重置
// 使用示例     All_PID_Init(&Angle_PID);
// 备注信息     注意需要考虑多个调用位置
//-------------------------------------------------------------------------------------------------------------------
void All_PID_Init(void)
{
	PID_Init(&Head__PID);
	PID_Init(&Track_PID);
	PID_Init(&Turn__PID);
	PID_Init(&Speed_PID);
	PID_Init(&Angle_PID);
	PID_Init(&Rate__PID);
	PID_Init(&Posi__PID);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     包括PID的变量重置
//-------------------------------------------------------------------------------------------------------------------
void BIG_Init(void)
{
	All_PID_Init();
	
	// 声光模块相关
	Delay_Timer_1 = 0;
	Delay_Timer_2 = 0;
	Delay_Timer_3 = 0;
	
	// 防止周期计时乱飞
	Time_Count1 = 0;
	Time_Count2 = 0;
	
	// 清零转向相关变量
	AvePWM = 0;
    DifPWM = 0;
	AveSpeed = 0;
    DifSpeed = 0;	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID计算
// 参数说明     PID_t *p	要传入的PID结构体变量名称
// 使用示例     PID_Update(&Angle_PID);
// 备注信息     包含PID优化逻辑，需要注意在"param_config.c"中赋予的初值
//-------------------------------------------------------------------------------------------------------------------

void PID_Update(PID_t *p)
{
	/* 获取本次误差和上次误差*/
	p->Error1 = p->Error0;					// 获取上次误差
	p->Error0 = p->Target - p->Actual;		// 获取本次误差，目标值减实际值，即为误差值
	
	float C = 1.0f;
	/*积分分离*/
	if ( fabs(p->Error0) > p->IntSepThresh )
	{
		C = 0.0f;
	}
	
	/* 外环误差积分（累加）*/
	/* 如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/* 因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/* 后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->Ki != 0)					// 如果Ki不为0
	{
		p->ErrorInt += p->Error0 * C;	// 进行误差积分
		// 积分限幅
		if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
		if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;	
	}
	else							// 否则
	{
		p->ErrorInt = 0;			// 误差积分直接归0
	}
	
	/* PID计算*/
	/* 使用位置式PID公式，计算得到输出值*/
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
	//		   + p->Kd * (p->Error0 - p->Error1); // 不使用微分先行
		   - p->Kd * (p->Actual - p->Actual1);    // 使用微分先行
	
	// 输出偏移
	if (p->Out > 0) {p->Out +=  p->OutOffset;}
	if (p->Out < 0) {p->Out -=  p->OutOffset;}
	
	// 输出限幅
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	// 限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	// 限制输出值最小为结构体指定的OutMin
	
	p->Actual1 = p->Actual;// 微分先行相关
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向环+速度环：PID算法封装
// 备注信息     可能需要注意在外部保证其他输入值的更新；
// 备注信息		PID总架构：[转向环] + [速度环 -> 角度环 -> 角速度环]
//-------------------------------------------------------------------------------------------------------------------
void PID_Calc_Speed_And_Turn(void)
{
	// 转向环PID计算		
//	if (fabsf(Angle_Result) < 10.0f)// 小车应该站稳了
//	{
		Turn__PID.Actual = DifSpeed;
		PID_Update(&Turn__PID);
		DifPWM = Turn__PID.Out;
//	}
//	// 看来没有
//	else 
//	{
//		PID_Init(&Turn__PID);
//	}

	// 速度环PID计算
	Speed_PID.Actual = AveSpeed;
	PID_Update(&Speed_PID);
	Angle_PID.Target = Speed_PID.Out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向环+速度环：PID算法封装
// 备注信息     可能需要注意在外部保证其他输入值的更新；
// 备注信息		PID总架构：[转向环] + [速度环 -> 角度环 -> 角速度环]
//-------------------------------------------------------------------------------------------------------------------
void PID_Calc_Angle_And_Rate(void)
{
	// 角度环PID计算
	Angle_PID.Actual = Angle_Result;
	PID_Update(&Angle_PID);
	Rate__PID.Target = Angle_PID.Out;
	
	// 角速度环PID计算
	Rate__PID.Actual = GyroRate_Result;
	PID_Update(&Rate__PID);
	AvePWM = - Rate__PID.Out;
	
	// 输出PWM换算
	LeftPWM  = AvePWM + DifPWM / 2.0f;
	RightPWM = AvePWM - DifPWM / 2.0f;

	// 输出限幅
	if (LeftPWM  > 8000){LeftPWM  = 8000;}else if (LeftPWM  < -8000){LeftPWM  = -8000;}
	if (RightPWM > 8000){RightPWM = 8000;}else if (RightPWM < -8000){RightPWM = -8000;}
	
	// 设置PWM
	motor_SetPWM(1, LeftPWM);
	motor_SetPWM(2, RightPWM);
}
