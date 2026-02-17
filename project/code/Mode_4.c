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


/*******************************************************************************************************************/
/*[S] 界面样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]模式内菜单界面
void Mode_4_Menu_UI(void)
{
	OLED_ShowString(8 , 0 , "Mode_4_Menu", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "Start", OLED_8X16);
	OLED_ShowString(10, 32, "Param", OLED_8X16);
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
int Mode_4_Running(void);

// [二级界面]模式内菜单界面

int Mode_4_Menu(void)
{
    // 模式菜单选项光标 标志位
    static uint8_t Mode_Menu_flag = 1;
    
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
            Mode_4_Running();
			
            // 返回后重新显示菜单
            OLED_Clear();
            Mode_4_Menu_UI();
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            OLED_Clear();
            Mode_4_Set_Param();
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_4_Menu_UI();
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

int Mode_4_Running(void)
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
	OLED_ShowString(0, 8 , "Kp:", OLED_6X8);
	OLED_ShowString(0, 16, "Ki:", OLED_6X8);
	OLED_ShowString(0, 24, "Kd:", OLED_6X8);
	OLED_ShowString(0, 32, "Tar:", OLED_6X8);
	OLED_ShowString(0, 40, "Act:", OLED_6X8);
	OLED_ShowString(0, 48, "Out:", OLED_6X8);
	OLED_ShowString(0, 56, "Int:", OLED_6X8);
	OLED_Update();
	
	// 清零pid积分等参数
	PID_Init(&Track_PID);
	PID_Init(&Turn__PID);
	PID_Init(&Speed_PID);
	PID_Init(&Angle_PID);
	PID_Init(&Rate__PID);
	
	// 防止周期计时乱飞
	Time_Count1 = 0;
	Time_Count2 = 0;
	
	// 清零编码器数值
	Get_Encoder1();
	Get_Encoder2();
	
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
			PID_Init(&Rate__PID);
			PID_Init(&Angle_PID);
			PID_Init(&Speed_PID);
			PID_Init(&Turn__PID);
			PID_Init(&Track_PID);	

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
			
			LeftSpeed  = Get_Encoder1() * 0.8f + Pre_LeftSpeed  * 0.2f;
			RightSpeed = Get_Encoder2() * 0.8f + Pre_RightSpeed * 0.2f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
			// 实际速度换算
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	// 实际平均速度
			DifSpeed = LeftSpeed - RightSpeed;			// 实际差分速度
			
			// 转向环PID计算		
			if (fabsf(Angle_Result) < 15.0f)// 小车应该站稳了
			{
				Turn__PID.Actual = DifSpeed;
				PID_Update(&Turn__PID);
				DifPWM = Turn__PID.Out;
			}
			// 看来没有
			else 
			{
				PID_Init(&Turn__PID);
			}
	
	
			// 速度环PID计算
			Speed_PID.Actual = AveSpeed;
			PID_Update(&Speed_PID);
			Angle_PID.Target = Speed_PID.Out;
			
		}

		
        if (Run_Flag)
		{			
			if (Time_Count1 >= 2)// 2 * 5 ms调控周期
			{
				Time_Count1 = 0;

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
				if (LeftPWM  > 9000){LeftPWM  = 9000;}else if (LeftPWM  < -9000){LeftPWM  = -9000;}
				if (RightPWM > 9000){RightPWM = 9000;}else if (RightPWM < -9000){RightPWM = -9000;}
				
				// 设置PWM
				motor_SetPWM(1, LeftPWM);
				motor_SetPWM(2, RightPWM);
			}						
			printf("%3.2f,%3.2f,%3.2f,%3.2f\r\n", Rate__PID.Target, GyroRate_Result, Angle_Result, Rate__PID.Out);
		}
		else
		{		
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
		}
			
		
//		OLED_Printf(24, 8 , OLED_6X8, "%4.2f ", Rate__PID.Kp);
//		OLED_Printf(24, 16, OLED_6X8, "%4.2f ", Rate__PID.Ki);
//		OLED_Printf(24, 24, OLED_6X8, "%4.2f ", Rate__PID.Kd);
//		OLED_Printf(24, 32, OLED_6X8, "%4.2f ", Rate__PID.Target);
//		OLED_Printf(24, 40, OLED_6X8, "%4.2f ", GyroRate_Result);
//		OLED_Printf(24, 48, OLED_6X8, "%4.2f ", Rate__PID.Out);
//		OLED_Printf(24, 56, OLED_6X8, "%4.2f ", Rate__PID.ErrorInt);
//		OLED_Update();
    }
}

/*******************************************************************************************************************/
/*[E] 小车运行 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
