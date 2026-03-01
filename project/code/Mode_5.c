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
#include "math.h"


#include "AI_tuning.h" // AI调参相关


/*******************************************************************************************************************/
/*[S] 界面样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]模式内菜单界面
void Mode_5_Menu_UI(void)
{
	OLED_ShowString(8 , 0 , "Mode_5_Menu[Ctrl]", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "Start", OLED_8X16);
	OLED_ShowString(10, 32, "Param", OLED_8X16);
}

// [三级界面]模式内参数设置界面
void Mode_5_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
			OLED_ShowString(8 , 0 , "M5_Param", OLED_6X8);
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

int Mode_5_Set_Param(void)
{
	Mode_5_Set_Param_UI(1);
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
int Mode_5_Running(void);

// [二级界面]模式内菜单界面

int Mode_5_Menu(void)
{
    // 模式菜单选项光标 标志位
    uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_5_Menu_UI();
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
            Mode_5_Running();
			
            // 返回后重新显示菜单
            OLED_Clear();
            Mode_5_Menu_UI();
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            OLED_Clear();
            Mode_5_Set_Param();
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_5_Menu_UI();
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

int Mode_5_Running(void)
{	 
	/* 初始化 AI 调参模块 */
//	ai_tuning_init();
		
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
	OLED_ShowString(0, 8 , "Yaw_Act:", OLED_6X8);
	OLED_ShowString(0, 16, "Yaw_Tar:", OLED_6X8);
	OLED_Update();
	
	// 清零pid积分等参数
	All_PID_Init();
	
	// 防止周期计时乱飞
	Time_Count1 = 0;
	Time_Count2 = 0;
	
	// 清零编码器数值
	Get_Encoder1();
	Get_Encoder2();
	
	// 航向角PID相关
	Head_PID_control_enable = 0;
	Yaw_Target = Yaw_Result;
	
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
			
			// 取反启动状态
			Run_Flag = !Run_Flag;			
			
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
		
		
		/* AI 调参模块接收处理 */
//		ai_tuning_handle_receive();
		
		
		/* 失控保护*/
		if (Angle_Result < - 50 || 50 < Angle_Result)
		{
			Run_Flag = 0;
			//强制停止（电机）运行
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
			
			OLED_ShowString(0, 0, "STOP", OLED_6X8);
			OLED_Update();
		}		
		
		
		/* 速度计算*/
		if (Time_Count2 >= 10)// 10 * 5 ms调控周期
		{
			Time_Count2 = 0;
			
			LeftSpeed  = Get_Encoder1() * 0.6f + Pre_LeftSpeed  * 0.4f;
			RightSpeed = Get_Encoder2() * 0.6f + Pre_RightSpeed * 0.4f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
			// 实际速度换算
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	// 实际平均速度
			DifSpeed = LeftSpeed - RightSpeed;			// 实际差分速度
			
			
			if (Run_Flag)
			{
				/* 航向角PID介入（挪用的时候注意航向角环输出取反给转向环）*/
				if (Head_PID_control_enable)
				{
//					if (fabs (Yaw_Target - Yaw_Result) < 2.0f){Head_PID_control_enable = 0;}				
					Head__PID.Target = Yaw_Target;
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
//		/* 输出数据用于AI调参 (timestamp, setpoint, input, pwm, error, angle, rate, speed, Kp, Ki, Kd) */
//		static uint32_t timestamp = 0;
//		timestamp += 10; /* 每次调用增加10ms (约100Hz) */
//		
//#if AI_TUNING_TARGET_PID_LOOP == 1
//		/* 角速度环 (Rate PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Rate__PID.Target, GyroRate_Result, AvePWM, Rate__PID.Target - GyroRate_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Rate__PID.Kp, Rate__PID.Ki, Rate__PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 2
//		/* 角度环 (Angle PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Angle_PID.Target, Angle_Result, AvePWM, Angle_PID.Target - Angle_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Angle_PID.Kp, Angle_PID.Ki, Angle_PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 3
//		/* 速度环 (Speed PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Speed_PID.Target, AveSpeed, AvePWM, Speed_PID.Target - AveSpeed,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Speed_PID.Kp, Speed_PID.Ki, Speed_PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 4
//		/* 转向环 (Turn PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Turn__PID.Target, DifSpeed, DifPWM, Turn__PID.Target - DifSpeed,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Turn__PID.Kp, Turn__PID.Ki, Turn__PID.Kd);
//#else
//		/* 默认输出角度环数据 */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Angle_PID.Target, Angle_Result, AvePWM, Angle_PID.Target - Angle_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Angle_PID.Kp, Angle_PID.Ki, Angle_PID.Kd);
//#endif			
		}
		else
		{		
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
		}
		OLED_Printf(48, 8 , OLED_6X8, "%3.2f", Yaw_Result);
//		OLED_Printf(48, 24 , OLED_6X8, "%3.2f", Speed_PID.Actual);
//		OLED_Printf(48, 32 , OLED_6X8, "%3.2f", Speed_PID.Target);
//		OLED_Printf(48, 40 , OLED_6X8, "%3.2f", Speed_PID.Out);
		if (Head_PID_control_enable)
		{
			OLED_Printf(48, 16, OLED_6X8, "%3.2f  ", Yaw_Target);
		}
		else
		{
			OLED_Printf(48, 16, OLED_6X8, "*%3.2f*", Yaw_Target);
		}
		OLED_Update();
    }
}

/*******************************************************************************************************************/
/*[E] 小车运行 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
