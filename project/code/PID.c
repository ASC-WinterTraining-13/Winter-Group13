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
		p->ErrorInt += p->Error0;	// 进行误差积分
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
// 函数简介     平衡车平衡PID算法
// 备注信息     注意在外部保证其他输入值的更新；
// 备注信息		输入值的更新时间可以不严格与计算周期同步，这里更加看重数据质量
// 备注信息		采用串级PID：转向环 -> 速度环 -> 角度环 -> 角速度环
// 备注信息		目前为调试版本，传入参数将决定pid环启用级数
//-------------------------------------------------------------------------------------------------------------------
void Balance_PID_Contorl(uint8_t Debug_Level)
{
	if (Debug_Level == 0){return;}
	
	// 失控保护
	if (Angle_Result < - 50.0f || 50.0f < Angle_Result)
	{
		motor_SetPWM(1, 0);
		motor_SetPWM(2, 0);
		
		return;
	}
	
	// 实际速度换算
	AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	//实际平均速度
	DifSpeed = LeftSpeed - RightSpeed;			//实际差分速度
	
	// 转向环PID计算
	if (Debug_Level >= 4)
	{
		Turn__PID.Actual = DifSpeed;
		PID_Update(&Turn__PID);
		DifPWM = Turn__PID.Out;
	}
	
	// 速度环PID计算
	if (Debug_Level >= 3)
	{
		Speed_PID.Actual = AveSpeed;
		PID_Update(&Speed_PID);
		Angle_PID.Target = Speed_PID.Out;
	}
	
	// 角度环PID计算
	if (Debug_Level >= 2)
	{
		Angle_PID.Actual = Angle_Result;
		PID_Update(&Angle_PID);
		Rate__PID.Target = Angle_PID.Out;
	}
	
	// 角速度环PID计算
	if (Debug_Level >= 1)
	{
		Rate__PID.Actual = GyroRate_Result;
		PID_Update(&Rate__PID);
		AvePWM = Rate__PID.Out;
	}
	
	// 输出PWM换算
	LeftPWM  = AvePWM + DifPWM / 2.0f;
	RightPWM = AvePWM - DifPWM / 2.0f;

	// 输出限幅
	if (LeftPWM  > 9000){LeftPWM = 9000;} else if (LeftPWM < -9000){LeftPWM = -9000;}
	if (RightPWM > 9000){RightPWM = 9000;}else if (RightPWM < -9000){RightPWM = -9000;}
	
	// 设置PWM
	motor_SetPWM(1, LeftPWM);
	motor_SetPWM(2, RightPWM);
}
