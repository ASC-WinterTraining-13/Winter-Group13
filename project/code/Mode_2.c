#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "mpu6050_Analysis.h"
#include "Encoder.h"
#include "motor.h"
#include "bluetooth_ch04_example.h"
#include "zf_device_bluetooth_ch04.h"
#include "PID.h"
#include "param_config.h"
#include "param_storage.h"
#include "Track_Sensor.h"
#include "BuzzerAndLED.h"


/*******************************************************************************************************************/
/*[S] 菜单样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 模式内界面
void Mode_2_Menu_UI(void)
{
    oled_show_string(0, 0, "Mode_2");
    oled_show_string(0, 2, "===");
    oled_show_string(2, 4, " Start");
    oled_show_string(2, 6, " Param");
}

// 模式内参数设置界面
void Mode_2_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
            oled_show_string(0, 0, "Param");
            oled_show_string(0, 1, "===");
            oled_show_string(2, 2, " Kp:");
            oled_show_string(2, 3, " Ki:");
            oled_show_string(2, 4, " Kd:");
            oled_show_float(28, 2, SPEED_KP, 4, 2);
            oled_show_float(28, 3, SPEED_KI, 4, 2);
            oled_show_float(28, 4, SPEED_KD, 4, 2);

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

void Set_Mode_2_Param(uint8_t Num)
{
    
    // 指向要修改的参数的指针
    float* current_param = NULL;
    float step_value = 0.0f;
    uint8_t row = 0;  // 数据对应的显示行号
    
    // 根据选项确定要修改的参数
    switch (Num)
    {
        case 1:  // Kp
            current_param = &SPEED_KP;
            step_value = PID_STEPS[1][0];
            row = 2;
            break;
            
        case 2:  // Ki
            current_param = &SPEED_KI;
            step_value = PID_STEPS[1][1];
            row = 3;
            break;
            
        case 3:  // Kd
            current_param = &SPEED_KD;
            step_value = PID_STEPS[1][2];
            row = 4;
            break;
    }
    
    oled_show_string(0, row, "=");
    
    while(1)
    {              
        /* 按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            *current_param += step_value;  // 增加参数
            oled_show_float(28, row, *current_param, 4, 2);  // 更新显示
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            *current_param -= step_value;  // 减少参数
            oled_show_float(28, row, *current_param, 4, 2);  // 更新显示
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
                 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
                key_clear_state(KEY_CONFIRM);
            if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
                key_clear_state(KEY_BACK);
            // 恢复光标为 ">"            
            oled_show_string(0, row, ">");
            Param_Save();
            break;  // 退出修改模式
        }
    }
}

//*******************************************************************************************************************/
/*[E] 参数更改 [E]--------------------------------------------------------------------------------------------------*/
/********************************************************************************************************************/


/********************************************************************************************************************/
/*[S] 交互界面 [S]--------------------------------------------------------------------------------------------------*/
/********************************************************************************************************************/

/* [模式内菜单子界面]*/

// 参数设置选项数量
#define OPT_NUM         3

int Mode_2_Set_Param(void)
{
    // 参数设置选项光标 标志位
    static uint8_t Param_flag = 1;
    
    // 显示
    oled_set_font(OLED_6X8_FONT);  
    Mode_2_Set_Param_UI(1);
    oled_show_string(0, 2, ">");
    
    while(1)
    {
        // 存储确认键被按下时Param_flag的值的临时变量，默认为无效值0
        uint8_t Param_flag_temp = 0;
        
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;  
                
        /* 按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_pressed = 1;
            key_clear_state(KEY_UP);
            Param_flag --;
            if (Param_flag < 1)Param_flag = OPT_NUM;    
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_pressed = 1;
            key_clear_state(KEY_DOWN);
            Param_flag ++;
            if (Param_flag > OPT_NUM)Param_flag = 1;    
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Param_flag_temp = Param_flag;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;
        }
        
        /* 数据更改模式*/
        if (Param_flag_temp)
        {
            Set_Mode_2_Param(Param_flag_temp);
        }        
        
        /* 显示更新*/
        // 判断界面是否需要更新
        if (key_pressed)
        {
            switch(Param_flag)
            {
                case 1:
                    oled_clear();
                    Mode_2_Set_Param_UI(1);
                    oled_show_string(0, 2, ">");
                
                    break;
                
                case 2:
                    oled_clear();
                    Mode_2_Set_Param_UI(1);
                    oled_show_string(0, 3, ">");
                
                    break;
                
                case 3:
                    oled_clear();
                    Mode_2_Set_Param_UI(1);
                    oled_show_string(0, 4, ">");
                    
                    break;                
            }
        }    
    }
}

// 方便模式内菜单母界面调用
int Mode_2_Running(void);

/* [模式内菜单母界面]*/

int Mode_2_Menu(void)
{
    // 模式菜单选项光标 标志位
    static uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_2_Menu_UI();
    oled_show_string(0, 4, ">");
    
    while(1)
    {
        // 存储确认键被按下时Mode_Menu_flag的值的临时变量，默认为无效值0
        uint8_t Mode_Menu_flag_temp = 0;
        
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;     
        
        /* 按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_pressed = 1;
            key_clear_state(KEY_UP);
            Mode_Menu_flag --;
            if (Mode_Menu_flag < 1)Mode_Menu_flag = 2;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_pressed = 1;
            key_clear_state(KEY_DOWN);
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
            Mode_2_Running();
            // 返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_2_Menu_UI();
            oled_show_string(0, 4, ">");
        }
        if (Mode_Menu_flag_temp == 2)
        {
            oled_clear();
            Mode_2_Set_Param();
            // 返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_2_Menu_UI();
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

// 模式2小车运行阶段
typedef enum {
    STATE_IDLE   = 0,          	// 空闲（未启动）
    STATE_A_TO_B = 1,        	// A→B：直线100cm
    STATE_B_TO_C = 2,        	// B→C：右半圆弧（半径60cm，180°）
    STATE_C_TO_D = 3,        	// C→D：直线100cm
    STATE_D_TO_A = 4,        	// D→A：左半圆弧（半径40cm，180°）
    STATE_STOP   = 5         	// 停车
} Mode_2_State;

Mode_2_State Mode_2_Cur_State = STATE_IDLE;

/*[模式内菜单子界面]*/

int Mode_2_Running(void)
{
	oled_set_font(OLED_6X8_FONT);

	oled_show_string(0, 1, "Tr_P:");
	oled_show_string(0, 2, "Tr_I:");
	oled_show_string(0, 3, "Tr_D:");
	oled_show_string(0, 4, "Error:");
	oled_show_string(0, 5, "Out:");
	oled_show_string(0, 6, "OUT_W:");
	oled_show_string(64, 6, "INN_W:");
	oled_show_string(64, 0, "Stage:");

	oled_show_string(0, 0, "Cali");
    
	// mpu6050零飘校准逻辑（此时请保持静止）
	MPU6050_Calibration_Start();
	while(1)  // lp零飘校准循环
    {
        if (MPU6050_Calibration_Check() == 0)  // 零飘校准完成
        {
            break;  // 跳出零飘校准循环，往下执行
        }
        
        // 可以考虑在这里操作OLED
        
        // 强制零飘校准退出
        if(KEY_SHORT_PRESS == key_get_state(KEY_BACK)) {
            key_clear_state(KEY_BACK);
            break;  // 正常退出零飘校准模式
        }
        
    }
	
	// 清零pid积分等参数
	PID_Init(&Angle_PID);
	PID_Init(&Speed_PID);
	PID_Init(&Turn_PID);
	PID_Init(&Track_PID);
	
	Run_Flag = 0;	
	oled_show_string(0, 0, "STOP");
	
    while(1)
    {  
		/* 按键处理*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))// 处理上键
        {
            key_clear_state(KEY_UP);
				
			// 发车
			Mode_2_Cur_State = STATE_IDLE;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))// 处理下键
        {
            key_clear_state(KEY_DOWN);
            
			// 发车
			Mode_2_Cur_State = STATE_IDLE;
        }
        else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))// 处理确认键
        {
            key_clear_state(KEY_CONFIRM);
            
			Param_Save();
			//清零pid积分等参数
			PID_Init(&Angle_PID);
			PID_Init(&Speed_PID);
			PID_Init(&Turn_PID);
			//更改启动状态
			Run_Flag = !Run_Flag;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))// 处理返回键
        {
            key_clear_state(KEY_BACK);            
			
			// 启停标志位置0
			Run_Flag = 0;
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			Param_Save();
			
            return 0;
        }		
		
		/*蓝牙模块*/
		bluetooth_ch04_handle_receive();	
		
		
		/* 失控保护*/
		if (Angle_Result < - 50 || 50 < Angle_Result)
		{
			Run_Flag = 0;
			//强制停止（电机）运行
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
		}
						
		
		/* 循迹处理*/
		float Error = Track_Sensor_Get_Error();
		
		switch(Track_Sensor_State)//是否在线
		{
			//  有线
			case TRACK_STATE_ON_LINE:
			{
				if (Mode_2_Cur_State == STATE_IDLE)
				{
					Speed_PID.Target = 20;
					Turn_PID.Target = 0;
				}					
				else if (Mode_2_Cur_State == STATE_A_TO_B)
				{
					Mode_2_Cur_State = STATE_B_TO_C;
					BuzzerAndLED_Delay_Timer = 200;
				}
				else if (Mode_2_Cur_State == STATE_B_TO_C)
				{
					Speed_PID.Target = 20;
					Track_PID.Actual = Error;
					PID_Update(&Track_PID);
					Turn_PID.Target = Track_PID.Out;
				}
				else if (Mode_2_Cur_State == STATE_C_TO_D)
				{
					Mode_2_Cur_State = STATE_D_TO_A;
					BuzzerAndLED_Delay_Timer = 200;
				}
				else if (Mode_2_Cur_State == STATE_D_TO_A)
				{
					Speed_PID.Target = 20;
					Track_PID.Actual = Error;
					PID_Update(&Track_PID);
					Turn_PID.Target = Track_PID.Out;
				}
				break;
			}
			// 无线
			case TRACK_STATE_OFF_LINE:
			{
				if (Mode_2_Cur_State == STATE_IDLE)
				{
					Mode_2_Cur_State = STATE_A_TO_B;
				}
				else if (Mode_2_Cur_State == STATE_A_TO_B) 
				{
					Speed_PID.Target = 25;
					Turn_PID.Target = 0;
				}
				else if (Mode_2_Cur_State == STATE_B_TO_C)
				{
					Mode_2_Cur_State = STATE_C_TO_D;
					BuzzerAndLED_Delay_Timer = 200;
				}
				else if (Mode_2_Cur_State == STATE_C_TO_D)
				{
					Speed_PID.Target = 25;
					Turn_PID.Target = 0;					
				}
				else if (Mode_2_Cur_State == STATE_D_TO_A)
				{
					Mode_2_Cur_State = STATE_STOP;
					BuzzerAndLED_Delay_Timer = 200;
					Speed_PID.Target = 0;
					Turn_PID.Target = 0;
				}
				
				break;
			}			
		}
		
		
		/* 声光模块*/
		if (BuzzerAndLED_Delay_Timer)
		{
			BuzzerAndLED_Promopt(1);
			BuzzerAndLED_Promopt(2);
		}
		else 
		{
			BuzzerAndLED_Promopt(3);
			BuzzerAndLED_Promopt(4);
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
		
//		bluetooth_ch04_printf("[plot,%f,%f]\r\n", Angle_PID.Target, Angle_PID.Actual);
		
		
		
		oled_show_float(30, 1, TRACK_KP, 5, 1);
		oled_show_float(30, 2, TRACK_KI, 3, 2);
		oled_show_float(30, 3, TRACK_KD, 5, 1);		
		oled_show_float(36, 4, Error, 3, 2);
		oled_show_float(24, 5, Track_PID.Out, 5, 2);
		oled_show_float(36, 6, outer_weight , 2, 1);
		oled_show_float(100, 6, inner_weight , 2, 1);
		oled_show_int(100, 0, Mode_2_Cur_State, 2);


    }
}

/*******************************************************************************************************************/
/*[E] 小车运行 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
