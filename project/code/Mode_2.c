#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "param_config.h"
#include "param_storage.h"
#include "mpu6050_Analysis.h"
#include "motor.h"
#include "Encoder.h"
#include "bluetooth_ch04_example.h"
#include "zf_device_bluetooth_ch04.h"
#include "PID.h"
#include "OLED.h"
#include "Track_Sensor.h"
#include "BuzzerAndLED.h"
#include "math.h"


/*******************************************************************************************************************/
/*[S] 界面样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]模式内菜单界面
void Mode_2_Menu_UI(void)
{
	OLED_ShowString(8 , 0 , "Mode_2_Menu[Path]", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "Start", OLED_8X16);
	OLED_ShowString(10, 32, "Param", OLED_8X16);
}

// [三级界面]模式内参数设置界面
void Mode_2_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
			OLED_ShowString(8 , 0 , "M2_Param", OLED_6X8);
			OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
			OLED_ShowString(10, 16, "NaN", OLED_6X8);
			
            break;
        }
    }
}
/*******************************************************************************************************************/
/*[E] 界面样式 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 参数更改 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [三级界面]模式内参数设置界面

int Mode_2_Set_Param(void)
{
	Mode_2_Set_Param_UI(1);
	OLED_ShowString(0, 16, ">", OLED_6X8);
	OLED_Update();
	
	while(1)
	{
		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
		{           
			key_clear_state(KEY_UP);
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{           
			key_clear_state(KEY_DOWN);
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
		{
			key_clear_state(KEY_CONFIRM);
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			// 返回上一级界面
			return 0;
		}
	}
}
/*******************************************************************************************************************/
/*[E] 参数更改 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 交互界面 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 函数提前声明
int Mode_2_Running(void);

// [二级界面]模式内菜单界面

int Mode_2_Menu(void)
{
    // 模式菜单选项光标 标志位
    uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_2_Menu_UI();
    OLED_ShowString(0, 16, ">", OLED_8X16);
	OLED_Update();
    
    while(1)
    {
        // 存储确认键被按下时Mode_Menu_flag的值的临时变量，默认为无效值0
        uint8_t Mode_Menu_flag_temp = 0;
        
        // 上/下按键是否被按下过，作为OLED刷新标志
        uint8_t key_pressed = 0;

        /* 按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {           
            key_clear_state(KEY_UP);
			key_pressed = 1;
            Mode_Menu_flag --;
            if (Mode_Menu_flag < 1)Mode_Menu_flag = 2;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {           
            key_clear_state(KEY_DOWN);
			key_pressed = 1;
            Mode_Menu_flag ++;
            if (Mode_Menu_flag > 2)Mode_Menu_flag = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Mode_Menu_flag_temp = Mode_Menu_flag;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;
        }
        
        /* 页面跳转*/
        if (Mode_Menu_flag_temp == 1)
        {
            OLED_Clear();
            Mode_2_Running();
			
            // 返回后重新显示菜单
            OLED_Clear();
            Mode_2_Menu_UI();
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            OLED_Clear();
            Mode_2_Set_Param();
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_2_Menu_UI();
            OLED_ShowString(0, 32, ">", OLED_8X16);  
			OLED_Update();
        }
        
        /* 显示更新*/
        if (key_pressed)
        {
            switch(Mode_Menu_flag)
            {
                case 1:
                {
                    OLED_ShowString(0, 16, ">", OLED_8X16);
					OLED_ShowString(0, 32, " ", OLED_8X16);
					OLED_Update();
					
                    break;
                }
				case 2:
                {
                    OLED_ShowString(0, 16, " ", OLED_8X16);
					OLED_ShowString(0, 32, ">", OLED_8X16);  
					OLED_Update();
					
                    break;
                }
            }
        }
    }
}

/*******************************************************************************************************************/
/*[E] 交互界面 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 小车运行 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [三级界面]模式小车运作界面

#define TRACK_SWITCH_COOLDOWN 600
// 模式二启用状态：1完整;0仅巡线
#define MODE_2_SET	1

uint8_t pre_Track_Sensor_State;

int Mode_2_Running(void)
{	 
		
	/* 半阻塞式MPU6050零飘校准逻辑(此时请保持静止)*/
	if (MPU6050_Calibration_Check() != 2)// 如果未校准
	{
		MPU6050_Calibration_Start();
		OLED_ShowString(0, 0, "CAli", OLED_6X8);
		OLED_Update();
	}
	// 半阻塞式零飘校准
	while(1)
    {
        if (MPU6050_Calibration_Check() == 2)  // 零飘校准完成
        {
            break;  // 结束零飘校准
        }      
        // 可以考虑在这里操作OLED，但请注意OLED对时间的占用
        
        // 强制零飘校准退出
        if(KEY_SHORT_PRESS == key_get_state(KEY_BACK)) {
            key_clear_state(KEY_BACK);
            break;  // 中止零飘校准
        }        
    }
	
	Run_Flag = 0;	
	OLED_ShowString(0, 0 , "STOP", OLED_6X8);
	OLED_ShowString(0, 8 , "Track:", OLED_6X8);
	OLED_ShowString(0, 16, "State:", OLED_6X8);
//	OLED_ShowString(0, 24, "Error:", OLED_6X8);
	OLED_ShowString(0, 32, "Yaw_A:", OLED_6X8);
	
	OLED_Update();
	
	// 模式2路径状态机
	// 原有的通用性质的Run_Flag仍然在使用，状态机更多是一个方便写代码的标志
typedef enum {
    STATE_IDLE   		= 0,          	// 空闲（未启动）
	STATE_BALANCE_ON 	= 1,			// 启用平衡控制
	STATE_SEEK_A		= 2,			// A：起点准备（考虑到可能会一开始，A点可能碰到线）（暂时认为离开线的一刻，为找到A点）
    STATE_A_TO_B 		= 3,        	// A→B：直线100cm
    STATE_B_TO_C 		= 4,        	// B→C：右半圆弧（半径40cm，180°）
    STATE_C_TO_D 		= 5,        	// C→D：直线100cm
    STATE_D_TO_A 		= 6,        	// D→A：左半圆弧（半径40cm，180°）
    STATE_STOP   		= 7,         	// 停车
	STATE_BALANCE_OFF	= 8				// 中止平衡控制
} Mode_2_State;

Mode_2_State Mode_2_Cur_State = STATE_IDLE;
	
	// 变量相关的重置部分整合
	BIG_Init();
	
	// 清零编码器数值
	Get_Encoder1();
	Get_Encoder2();
	Encoder_Left = 0;
	Encoder_Right = 0;

	BUZ_SET(0);
	LED_SET(0);

	// 平滑相关的算法
	float Error_filtered = 0;

	// 记录弯道行驶距离（左右编码器原始值直接累加）
	uint16_t Turn_Encoder_Accum = 0;
// 标志位，标记当前是否为转弯状态
	uint8_t Turn_State_flag = 0;
	
	// 正式运行
    while(1)
    {  
		/* 按键处理*/
//        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))// 上键
//        {
//            key_clear_state(KEY_UP);
//        }
//        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))// 下键
//        {
//            key_clear_state(KEY_DOWN);
//        }
//        else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))// 确认键
        {
            key_clear_state(KEY_CONFIRM);
#if MODE_2_SET	== 1	
			/* 运行状态切换逻辑*/
			if (Mode_2_Cur_State == STATE_IDLE)// 开启平衡控制
			{
				Mode_2_Cur_State = STATE_BALANCE_ON;
				Head_PID_control_enable = 0;
				Run_Flag = 1;
				OLED_ShowString(36, 16, "BAL ", OLED_6X8);
				OLED_Update();
			}
			else if (Mode_2_Cur_State == STATE_BALANCE_ON)// 起点（A点）准备
			{
				Mode_2_Cur_State = STATE_SEEK_A;
				OLED_ShowString(36, 16, "A:??", OLED_6X8);
			}
			else if (Mode_2_Cur_State == STATE_SEEK_A)// 直接开始跑车
			{
				Mode_2_Cur_State = STATE_A_TO_B;
				Yaw_Target = Yaw_Result;
				Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;  
				OLED_ShowString(36, 16, "A->B", OLED_6X8);
				OLED_Update();
			}
			else// 其他阶段，按下确认键则直接停止
			{
				Mode_2_Cur_State = STATE_IDLE;
				Head_PID_control_enable = 0;
				motor_SetPWM(1, 0);
				motor_SetPWM(2, 0);
				DifPWM  = 0;
				Run_Flag = 0;
				OLED_ShowString(36, 16, "IDLE", OLED_6X8);
				OLED_Update();
			}
#else
			Run_Flag = 1 - Run_Flag;
#endif
			
			
			// PID参数存储
			Param_Save();
			
			// 清零pid积分等参数
			All_PID_Init();	

			if (Run_Flag) {OLED_ShowString(0, 0, "Run ", OLED_6X8);OLED_Update();}
			else {OLED_ShowString(0, 0, "STOP", OLED_6X8);OLED_Update();}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))// 返回键
        {
            key_clear_state(KEY_BACK);
			
			// 启停标志位置0
			Run_Flag = 0;
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
			
			// 关闭声光
			BUZ_SET(0);
            LED_SET(0);
			
			// PID参数存储
			Param_Save();

			// 返回上一级菜单
            return 0;
        }
		
		
		/* mpu6050数据接收与解析*/
		if (mpu6050_analysis_enable)
		{
			mpu6050_get_data();
			mpu6050_analysis_enable = 0;
			MPU6050_Analysis();
		}
		
		
		/* 蓝牙模块*/
		bluetooth_ch04_handle_receive();			
		
		
		/* 失控保护*/
		if (Angle_Result < - 50 || 50 < Angle_Result)
		{
			Mode_2_Cur_State = STATE_IDLE;
			Head_PID_control_enable = 0;
			Run_Flag = 0;
			//强制停止（电机）运行
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
			
			OLED_ShowString(0, 0, "STOP", OLED_6X8);
			OLED_Update();
		}		
		
		
				
		/* 路径处理*/
        float Error = Track_Sensor_Get_Error();
		Error_filtered = 0.6 * Error + 0.4 * Error_filtered;
		
#if MODE_2_SET == 0
		
		// 非任务模式：单巡线调试模式（麻烦暴力使用注释来更改模式）
		switch(Track_Sensor_State)//是否在线
		{
			//  在线
			case TRACK_STATE_ON_LINE:
			{
//				OLED_ShowString(36, 8 , "ON ", OLED_6X8);
				
				Speed_PID.Target = 30;
				Track_PID.Actual = Error_filtered;
				PID_Update(&Track_PID);
				Turn__PID.Target = Track_PID.Out;
				
				if (pre_Track_Sensor_State != Track_Sensor_State){bluetooth_ch04_printf("N\r\n");}
				pre_Track_Sensor_State = Track_Sensor_State;
			}
			//  掉线
			case TRACK_STATE_OFF_LINE:
			{
//				OLED_ShowString(36, 8 , "OFF", OLED_6X8);
				
				if (pre_Track_Sensor_State != Track_Sensor_State){bluetooth_ch04_printf("F\r\n");}
				pre_Track_Sensor_State = Track_Sensor_State;
			}
		}
//		OLED_Update();
//		bluetooth_ch04_printf("[plot,%2.1f]\r\n", Turn__PID.Out);
		
#else
		
		// 任务模式	
//		OLED_Printf(36, 24, OLED_6X8, "%2.1f", Error);
		OLED_Printf(36, 32, OLED_6X8, "%2.1f", Yaw_Result);
        switch(Track_Sensor_State)//是否在线
        {
			case TRACK_STATE_ON_LINE:
			{
				OLED_ShowString(36, 8 , "ON ", OLED_6X8);
				
				// ========== 状态切换判定（受冷却影响）==========
				if (Delay_Timer_2 == 0) 
				{
					if (Mode_2_Cur_State == STATE_A_TO_B)
					{
						// 到达B
						Head_PID_control_enable = 0;
						Mode_2_Cur_State = STATE_B_TO_C;
						OLED_ShowString(36, 16, "B->C", OLED_6X8);
//						bluetooth_ch04_printf("B\r\n");
						Delay_Timer_1 = 50;// 声光触发
						Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
						Turn_Encoder_Accum = 0;
						Turn_State_flag = 1;// 弯道行驶距离累积开启
					}
					else if (Mode_2_Cur_State == STATE_C_TO_D)
					{
						// 到达D
						Head_PID_control_enable = 0;
						Mode_2_Cur_State = STATE_D_TO_A;
						OLED_ShowString(36, 16, "D->A", OLED_6X8);
//						bluetooth_ch04_printf("D\r\n");
						Delay_Timer_1 = 50;// 声光触发
						Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
						Turn_Encoder_Accum = 0;
						Turn_State_flag = 1;// 弯道行驶距离累积开启
					}
				}
				
				// ========== 巡线执行（不受冷却影响）==========
				if (Mode_2_Cur_State == STATE_SEEK_A || 
					Mode_2_Cur_State == STATE_B_TO_C || 
					Mode_2_Cur_State == STATE_D_TO_A)
				{
					Speed_PID.Target = 30;
					Track_PID.Actual = Error_filtered;
					PID_Update(&Track_PID);
					Turn__PID.Target = Track_PID.Out;
				}
				
				break;
			}

			case TRACK_STATE_OFF_LINE:
			{
				OLED_ShowString(36, 8 , "OFF", OLED_6X8);
				
				// ========== 状态切换判定（受冷却影响）==========
				if (Delay_Timer_2 == 0) 
				{
					if (Mode_2_Cur_State == STATE_SEEK_A)
					{
						// 到达A
						Mode_2_Cur_State = STATE_A_TO_B;
						Yaw_Target = Yaw_Result;
						OLED_ShowString(36, 16, "A->B", OLED_6X8);
//						bluetooth_ch04_printf("A\r\n");
						Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
					}
					else if (Mode_2_Cur_State == STATE_B_TO_C && 
							 (Yaw_Target - 185 <= Yaw_Result && Yaw_Result <= -175) &&
							Turn_Encoder_Accum >= 5400)
					{
						// 到达C
						Turn_State_flag = 0;
						Mode_2_Cur_State = STATE_C_TO_D;
						OLED_ShowString(36, 16, "C->D", OLED_6X8);
//						bluetooth_ch04_printf("C\r\n");
						Delay_Timer_1 = 50;// 声光触发
						Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
					}
					else if (Mode_2_Cur_State == STATE_D_TO_A && 
							 (Yaw_Target - 365 <= Yaw_Result && Yaw_Result <= -355) &&
							Turn_Encoder_Accum >= 5400)
					{
						// 到达A
						Turn_State_flag = 0;
						Mode_2_Cur_State = STATE_STOP;
						Speed_PID.Target = 0;
						Turn__PID.Target = 0;
						OLED_ShowString(36, 16, "STOP", OLED_6X8);
//						bluetooth_ch04_printf("P\r\n");
						Delay_Timer_1 = 50;// 声光触发
					}
				}
				
				// ========== 航向角控制（不受冷却影响）==========
				if (Mode_2_Cur_State == STATE_A_TO_B)
				{
					Speed_PID.Target = 30;
					Head__PID.Target = Yaw_Target;
					Head_PID_control_enable = 1;
				}
				else if (Mode_2_Cur_State == STATE_C_TO_D)
				{
					Speed_PID.Target = 30;
					Head__PID.Target = Yaw_Target - 180;
					Head_PID_control_enable = 1;
				}
				
				break;
			}
        }	
		OLED_Update();
		
#endif
		
        
        /* 声光模块*/
        if (Delay_Timer_1)// 基于5ms定时器的分频计数器
        {
            BUZ_SET(1);
            LED_SET(1);
        }
        else 
        {
            BUZ_SET(0);
            LED_SET(0);
        }


		
		/* 速度计算*/
		if (Time_Count2 >= 10)// 10 * 5 ms调控周期
		{
			Time_Count2 = 0;
			
			// 编码器数据接收与处理
			Encoder_Left  = Get_Encoder1();
			Encoder_Right = Get_Encoder2();
			LeftSpeed  = Encoder_Left * 0.6f + Pre_LeftSpeed  * 0.4f;
			RightSpeed = Encoder_Right * 0.6f + Pre_RightSpeed * 0.4f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
			// 弯道行驶距离累积
			if (Turn_State_flag){Turn_Encoder_Accum += Encoder_Left + Encoder_Right;}
			
			// 实际速度换算
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	// 实际平均速度
			DifSpeed = LeftSpeed - RightSpeed;			// 实际差分速度
			
			if (Run_Flag)
			{
				/* 航向角PID介入（挪用的时候注意航向角环输出取反给转向环）*/
				if (Head_PID_control_enable)
				{
//					if (fabs (Yaw_Target - Yaw_Result) < 2.0f){Head_PID_control_enable = 0;}
					Head__PID.Actual = Yaw_Result;
					PID_Update(&Head__PID);
					Turn__PID.Target = - Head__PID.Out;
				}
				
				/* 转向环+速度环PID计算*/
				PID_Calc_Speed_And_Turn();
			}
		}

		
        if (Run_Flag)
		{			
			if (Time_Count1 >= 2)// 2 * 5 ms调控周期
			{
				Time_Count1 = 0;
				
				/* 角度环+角速度环PID计算（包括PWM设置）*/
				PID_Calc_Angle_And_Rate();
			}						
		}
		else
		{		
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
		}
    }
}

/*******************************************************************************************************************/
/*[E] 小车运行 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
