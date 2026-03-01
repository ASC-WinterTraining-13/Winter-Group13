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
#include "navigation.h"
#include "navi_flash.h"


/*******************************************************************************************************************/
/*[S] 界面样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]模式内菜单界面
void Mode_4_Menu_UI(void)
{
	OLED_ShowString(8 , 0 , "Mode_4_Menu[Navi]", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "Record", OLED_8X16);
	OLED_ShowString(10, 32, "Replay", OLED_8X16);
	OLED_ShowString(10, 48, "Param", OLED_8X16);
}

// [三级界面]模式内参数设置界面
void Mode_4_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
			OLED_ShowString(8 , 0 , "M4_Param", OLED_6X8);
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

int Mode_4_Set_Param(void)
{
	Mode_4_Set_Param_UI(1);
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
int Mode_4_Running(uint8 navi_mode);

// [二级界面]模式内菜单界面

int Mode_4_Menu(void)
{
    // 模式菜单选项光标 标志位
    uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_4_Menu_UI();
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
            if (Mode_Menu_flag < 1)Mode_Menu_flag = 3;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {           
            key_clear_state(KEY_DOWN);
			key_pressed = 1;
            Mode_Menu_flag ++;
            if (Mode_Menu_flag > 3)Mode_Menu_flag = 1;
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
            Mode_4_Running(1); // 1 = 记录模式
			
            // 返回后重新显示菜单
            OLED_Clear();
            Mode_4_Menu_UI();
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            OLED_Clear();
            Mode_4_Running(3); // 3 = 回放模式
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_4_Menu_UI();
            OLED_ShowString(0, 32, ">", OLED_8X16);  
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 3)
        {
            OLED_Clear();
            Mode_4_Set_Param();
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_4_Menu_UI();
            OLED_ShowString(0, 48, ">", OLED_8X16);  
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
					OLED_ShowString(0, 48, " ", OLED_8X16);
					OLED_Update();
					
                    break;
                }
				case 2:
                {
                    OLED_ShowString(0, 16, " ", OLED_8X16);
					OLED_ShowString(0, 32, ">", OLED_8X16);  
					OLED_ShowString(0, 48, " ", OLED_8X16);
					OLED_Update();
					
                    break;
                }
				case 3:
                {
                    OLED_ShowString(0, 16, " ", OLED_8X16);
					OLED_ShowString(0, 32, " ", OLED_8X16);
					OLED_ShowString(0, 48, ">", OLED_8X16);  
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

int Mode_4_Running(uint8 navi_mode)
{	 
	// 初始化惯性导航系统
	Init_Nag();
	N.Nag_SystemRun_Index = 0; // 先停止惯导，等待状态切换
	
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
	
	// 状态机：0=Balance:STOP, 1=Balance:RUN, 2=录制/回放中, 3=停止录制/回放(保持平衡)
	uint8 state = 0;
	uint8 balance_enable = 0;
	uint8 navi_enable = 0;
	
	// 第一行显示Balance状态
	OLED_ShowString(0, 0, "Bal:STOP", OLED_6X8);
	// 第二行显示录制/回放状态
	if (navi_mode == 1) {
		OLED_ShowString(0, 8, "REC:STOP", OLED_6X8);
	} else if (navi_mode == 3) {
		OLED_ShowString(0, 8, "PLY:STOP", OLED_6X8);
	}
	OLED_ShowString(0, 16, "Yaw_Nav:", OLED_6X8);
	OLED_ShowString(0, 24, "Yaw_Tar:", OLED_6X8);
	OLED_ShowString(0, 32, "Idx:", OLED_6X8);
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
			
			// 状态机循环：0→1→2→3→0
			state = (state + 1) % 4;
			
			// 根据状态更新标志位
			switch(state)
			{
				case 0: // Balance:STOP
					balance_enable = 0;
					navi_enable = 0;
					N.Nag_SystemRun_Index = 0;
					break;
				case 1: // Balance:RUN
					balance_enable = 1;
					navi_enable = 0;
					N.Nag_SystemRun_Index = 0;
					break;
				case 2: // 开始录制/回放
					balance_enable = 1;
					navi_enable = 1;
					N.Nag_SystemRun_Index = navi_mode;
					// 记录此时的yaw角作为偏移量
					N.Yaw_Dif = Yaw_Result;
					// 清零pid积分等参数
					All_PID_Init();
					// 回放模式：先预加载记录长度与首个目标角
					if (navi_mode == 3)
					{
						if (Nag_Replay_Start())
						{
							navi_enable = 0;
							N.Nag_SystemRun_Index = 0;
							Speed_PID.Target = 0;
							Turn__PID.Target = 0;
							Head_PID_control_enable = 0;
						}
						else
						{
							// 回放启动预热：5ms计数基准，延时约30ms后再接入航向角控制
							Delay_Timer_1 = 6;
						}
					}
					break;
				case 3: // 停止录制/回放，保持平衡
					balance_enable = 1;
					navi_enable = 0;
					N.Nag_SystemRun_Index = 0;
					// 如果是记录模式，写入最后一页数据
					if (navi_mode == 1) {
						N.End_f = 1;
						flash_Navi_Write();
					}
					break;
			}
			
			// PID参数存储
			Param_Save();
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))// 返回键
        {
            key_clear_state(KEY_BACK);
			
			// 如果是记录模式且正在录制，先结束记录
			if (navi_mode == 1 && navi_enable) {
				N.End_f = 1;
				flash_Navi_Write();
			}
			
			// 停止所有运行
			balance_enable = 0;
			navi_enable = 0;
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
		
		
		/* 失控保护*/
		if (Angle_Result < - 50 || 50 < Angle_Result)
		{
			balance_enable = 0;
			navi_enable = 0;
			N.Nag_SystemRun_Index = 0;
			// 如果是记录模式，结束记录
			if (navi_mode == 1) {
				N.End_f = 1;
				flash_Navi_Write();
			}
			//强制停止（电机）运行
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
			
			OLED_Update();
		}
		
		/* 回放自动停止检测*/
		if (navi_mode == 3 && navi_enable && N.Nag_Stop_f)
		{
			navi_enable = 0;
			N.Nag_SystemRun_Index = 0;
			// 保持平衡，不停止平衡控制
			Speed_PID.Target = 0;
			Turn__PID.Target = 0;
			Head_PID_control_enable = 0;
			
			OLED_Update();
		}		
		
		
		/* 速度计算*/
		if (Time_Count2 >= 10)// 10 * 5 ms调控周期
		{
			Time_Count2 = 0;
			float nav_yaw = 0.0f;
			if (navi_enable)
			{
				nav_yaw = Yaw_Result - N.Yaw_Dif;
				if (nav_yaw > 180.0f) nav_yaw -= 360.0f;
				else if (nav_yaw < -180.0f) nav_yaw += 360.0f;
			}
			
			LeftSpeed  = Get_Encoder1() * 0.6f + Pre_LeftSpeed  * 0.4f;
			RightSpeed = Get_Encoder2() * 0.6f + Pre_RightSpeed * 0.4f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
			// 实际速度换算
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	// 实际平均速度
			DifSpeed = LeftSpeed - RightSpeed;			// 实际差分速度
			
			if (balance_enable)
			{
				// 记录模式：手推，不设置速度目标，只保持平衡
				if (navi_mode == 1)
				{
					Speed_PID.Target = 0;
					Turn__PID.Target = 0;
				}
				// 回放模式：自动行驶，设置速度和航向角控制
				else if (navi_mode == 3)
				{
					if (navi_enable) {
						if (Delay_Timer_1 == 0)
						{
							Speed_PID.Target = 20;
							Head_PID_control_enable = 1;
							Yaw_Target = N.Angle_Run;
							
							/* 航向角PID介入（航向角环输出取反给转向环）*/
							Head__PID.Target = Yaw_Target;
							Head__PID.Actual = Yaw_Result;
							PID_Update(&Head__PID);
							Turn__PID.Target = - Head__PID.Out;
						}
						else
						{
							Speed_PID.Target = 0;
							Head_PID_control_enable = 0;
							Turn__PID.Target = 0;
						}
					} else {
						Speed_PID.Target = 0;
						Turn__PID.Target = 0;
					}
				}
				
				/* 转向环+速度环PID计算*/
				PID_Calc_Speed_And_Turn();
			}
			
			// 更新OLED显示
			OLED_Printf(48, 16, OLED_6X8, "%3.2f", nav_yaw);
			if (navi_mode == 3 && navi_enable) {
				float yaw_tar_rel = N.Angle_Run - N.Yaw_Dif;
				if (yaw_tar_rel > 180.0f) yaw_tar_rel -= 360.0f;
				else if (yaw_tar_rel < -180.0f) yaw_tar_rel += 360.0f;
				OLED_Printf(48, 24, OLED_6X8, "%3.2f  ", yaw_tar_rel);
			} else {
				OLED_ShowString(48, 24, "#####", OLED_6X8);
			}
			OLED_ShowNum(32, 32, navi_mode == 1 ? N.Save_index : N.Run_index, 4, OLED_6X8);
			
			// 更新Balance状态显示（第一行）
			if (balance_enable) {
				OLED_ShowString(0, 0, "Bal:RUN ", OLED_6X8);
			} else {
				OLED_ShowString(0, 0, "Bal:STOP", OLED_6X8);
			}
			
			// 更新录制/回放状态显示（第二行）
			if (navi_mode == 1) {
				if (navi_enable) {
					OLED_ShowString(0, 8, "REC:RUN ", OLED_6X8);
				} else {
					OLED_ShowString(0, 8, "REC:STOP", OLED_6X8);
				}
			} else if (navi_mode == 3) {
				if (navi_enable) {
					OLED_ShowString(0, 8, "PLY:RUN ", OLED_6X8);
				} else {
					OLED_ShowString(0, 8, "PLY:STOP", OLED_6X8);
				}
			}
			
			OLED_Update();
		}

		
        if (balance_enable)
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
