#include "zf_device_oled.h"
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


/*******************************************************************************************************************/
/*[S] 菜单样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]模式内菜单界面
void Mode_1_Menu_UI(void)
{
    oled_show_string(0, 0, "M1_Menu");
    oled_show_string(0, 2, "===");
    oled_show_string(2, 4, " Start");
	oled_show_string(2, 6, " Param");
}

// [三级界面]模式内参数设置界面
void Mode_1_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
            oled_show_string(0, 0, "M1_Param");
            oled_show_string(0, 1, "===");
            oled_show_string(0, 2, " NaN");
			
            break;
        }
    }
}
/*******************************************************************************************************************/
/*[E] 菜单样式 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 参数更改 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int Mode_1_Set_Param(void)
{
	oled_clear();
	oled_set_font(OLED_6X8_FONT);
	Mode_1_Set_Param_UI(1);
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
int Mode_1_Running(void);

/* [模式内菜单母界面]*/

int Mode_1_Menu(void)
{
    // 模式菜单选项光标 标志位
    static uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_1_Menu_UI();
    oled_show_string(0, 4, ">");
    
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
            oled_clear();
            Mode_1_Running();
            // 返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_1_Menu_UI();
            oled_show_string(0, 4, ">");
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            oled_clear();
            Mode_1_Set_Param();
            //返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_1_Menu_UI();
            oled_show_string(0, 6, ">");
        }
        
        /* 显示更新*/
        if (key_pressed)
        {
            switch(Mode_Menu_flag)
            {
                case 1:
                {
                    oled_show_string(0, 4, ">");
					oled_show_string(0, 6, " ");
                    break;
                }
				case 2:
                {
                    oled_show_string(0, 4, " ");
                    oled_show_string(0, 6, ">");                   
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

/*[模式内菜单子界面]*/

int Mode_1_Running(void)
{	
	oled_set_font(OLED_6X8_FONT);
    
	oled_show_string(0, 0, "Cail");
	
	/* 半阻塞式MPU6050零飘校准逻辑(此时请保持静止)*/
	MPU6050_Calibration_Start();	
	while(1)
    {
        if (MPU6050_Calibration_Check() == 0)  // 零飘校准完成
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
	oled_show_string(0, 0, "STOP");
	
	// 清零pid积分等参数
	PID_Init(&Rate__PID);
	PID_Init(&Angle_PID);
	PID_Init(&Speed_PID);
	PID_Init(&Turn__PID);
	PID_Init(&Track_PID);
	
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
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))// 返回键
        {
            key_clear_state(KEY_BACK);
			
			// 启停标志位置0
			Run_Flag = 0;
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			
			// PID参数存储
			Param_Save();

			// 返回上一级菜单
            return 0;
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
		}		
		
		
		/* 速度计算*/
		if (Time_Count2 > 20)// 20 * 5 ms调控周期
		{
			Time_Count2 = 0;
			
			LeftSpeed  = Get_Encoder1() * 0.6f + Pre_LeftSpeed  * 0.4f;
			RightSpeed = Get_Encoder2() * 0.6f + Pre_RightSpeed * 0.4f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
		}

		
		/* PID*/
        if (Run_Flag)
		{			
			oled_show_string(0, 0, "Run ");
			if (Time_Count1 > 2)// 2 * 5 ms调控周期
			{
				Time_Count1 = 0;
				// PID调控
				Balance_PID_Contorl();
			}						
		}
		else
		{
			oled_show_string(0, 0, "STOP");
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
		}
			
		
		/* mpu6050数据接收与解析*/
		if (mpu6050_analysis_enable)
		{
			mpu6050_get_data();
			mpu6050_analysis_enable = 0;
			MPU6050_Analysis();
		}
    }
}

/*******************************************************************************************************************/
/*[E] 小车运行 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
