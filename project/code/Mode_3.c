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
void Mode_3_Menu_UI(void)
{
	OLED_ShowString(8 , 0 , "Mode_3_Menu[Path]", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "Start", OLED_8X16);
	OLED_ShowString(10, 32, "Param", OLED_8X16);
}

// [三级界面]模式内参数设置界面
void Mode_3_Set_Param_UI(uint8_t Page)
{
    switch(Page)
	{
        
        // 第一页
        case 1:
        {
			OLED_ShowString(8 , 0 , "M3_Param", OLED_6X8);
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

int Mode_3_Set_Param(void)
{
	Mode_3_Set_Param_UI(1);
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
int Mode_3_Running(void);

// [二级界面]模式内菜单界面

int Mode_3_Menu(void)
{
    // 模式菜单选项光标 标志位
    uint8_t Mode_Menu_flag = 1;
    
    // 显示
    Mode_3_Menu_UI();
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
            Mode_3_Running();
			
            // 返回后重新显示菜单
            OLED_Clear();
            Mode_3_Menu_UI();
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
        }
		else if (Mode_Menu_flag_temp == 2)
        {
            OLED_Clear();
            Mode_3_Set_Param();
			
            //返回后重新显示菜单
            OLED_Clear();
            Mode_3_Menu_UI();
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

// 点位切换冷却时间
#define TRACK_SWITCH_COOLDOWN 600

int Mode_3_Running(void)
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
//	OLED_ShowString(0, 8 , "Track:", OLED_6X8);
	OLED_ShowString(0, 16, "State:", OLED_6X8);
//	OLED_ShowString(0, 24, "Error:", OLED_6X8);
	OLED_ShowString(0, 32, "Yaw_A:", OLED_6X8);

	OLED_Update();
	
	// 模式3路径状态机
	// 原有的通用性质的Run_Flag仍然在起使能平衡车四环的作用
	// 状态机更多起方便写代码的作用
	typedef enum {
		STATE_IDLE   		= 0,          	// 空闲（未启动）
		STATE_BALANCE_ON 	= 1,			// 启用平衡控制
		STATE_PREP			= 2,			// A：起点准备（考虑从A点后的引导线上发车）
		STATE_A_TO_C 		= 3,        	// A→C："对角"直线
		STATE_C_TO_B 		= 4,        	// C→B：右半圆弧（半径40cm，180°）
		STATE_B_TO_D 		= 5,        	// B→D："对角"直线
		STATE_D_TO_A 		= 6,        	// D→A：左半圆弧（半径40cm，180°）
		STATE_STOP   		= 7,         	// 停车
		STATE_BALANCE_OFF	= 8				// 中止平衡控制
	} Mode_3_State;

	Mode_3_State Mode_3_Cur_State = STATE_IDLE;
	
	
	// 子状态状态机（A→C："对角"直线||B→D："对角"直线）
	typedef enum {
		INNER_STATE_TURN_RIGHT    = 0,	// 右转（相对子状态开始时）
		INNER_STATE_GO_STRAIGHT_1 = 1,	// 直线行驶一
		INNER_STATE_TURN_LEFT     = 2,	// 左转
		INNER_STATE_GO_STRAIGHT_2 = 3	// 直线行驶二
	} Inner_State;
	Inner_State Cur_Inner_State = INNER_STATE_TURN_RIGHT;
	
	// 圈数累计
	uint8_t Mode_3_Turns_Count = 0;
	
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
	float Track_Out_filtered = 0;

	// 记录弯道行驶距离（左右编码器原始值直接累加）
	uint16_t Turn_Encoder_Accum = 0;
	// 标志位，标记当前是否为弯道行驶状态：0/1
	uint8_t Turn_State_flag = 0;
	// 记录直线行驶距离（左右编码器原始值直接累加）（当然指的不是所有的直线，这里是指从A→C，B→D中考虑的一段路径）
	uint16_t Straight_Encoder_Accum = 0;
	// 标志位，标记当前是否为直线行驶状态（当然指的不是所有的直线，这里是指从A→C，B→D中考虑的一段路径）：0/1
	uint8_t Straight_State_flag = 0;
	
	// 循迹周期处理相关
	// 0 该周期完成第一次读取之前
	// 1 该周期完成第二次读取之前
	// 2 完成两次读取
	// （干脆理解为周期内读取计数也行）
	uint8_t Track_Scan_flag = 0;
	
	
	// 正式开始运行模式代码
    while(1)
    {  
		
		
		/*======================================================*/
		/*[按键处理]*********************************************/
		/*======================================================*/
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
			
			/* 运行状态切换逻辑*/
			if (Mode_3_Cur_State == STATE_IDLE)// 带"准备"发车
			{
				Mode_3_Cur_State = STATE_PREP;
				Mode_3_Turns_Count = 0;// 重置圈数
				Turn_State_flag     = 0;// 弯道行驶标志位 置0
				Straight_State_flag = 0;// 直线行驶标志位 置0
				Head_PID_control_enable = 0;// 关闭航向角环PID使能
				Run_Flag = 1;			
				Speed_PID.Target = 35;
			}
			else if (Mode_3_Cur_State == STATE_PREP)// 直接发车
			{
				Mode_3_Cur_State = STATE_A_TO_C;
				Cur_Inner_State = INNER_STATE_TURN_RIGHT;
				Mode_3_Turns_Count = 0;// 重置圈数
				Turn_State_flag     = 0;// 弯道行驶标志位 置0
				Straight_State_flag = 0;// 直线行驶标志位 置0
				Head_PID_control_enable = 0;// 关闭航向角环PID使能
				Run_Flag = 1;			
				Speed_PID.Target = 0;
				Yaw_Target = Yaw_Result;// 标定发车Yaw角，这将作为后续行驶的基准
			}
			else// 停车并重置状态
			{
				Mode_3_Cur_State = STATE_IDLE;
				Mode_3_Turns_Count = 0;// 重置圈数				
				Turn_State_flag     = 0;// 弯道行驶标志位 置0
				Straight_State_flag = 0;// 直线行驶标志位 置0
				Head_PID_control_enable = 0;// 关闭航向角环PID使能
				Run_Flag = 0;
				BIG_Init();
			}
						
			// PID参数存储
			Param_Save();
			
			// 清零pid积分等参数
			BIG_Init();	

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
			Head_PID_control_enable = 0;// 关闭航向角环PID使能
			
			// 关闭声光
			BUZ_SET(0);
            LED_SET(0);
			
			// PID参数存储
			Param_Save();

			// 返回上一级菜单
            return 0;
        }
		/*======================================================*/
		/*********************************************[按键处理]*/
		/*======================================================*/
		
		
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
			if (Run_Flag){OLED_ShowString(36, 16, "BAOF", OLED_6X8);// BAOF:BALANCE OFF
						  OLED_ShowString(0 , 0 , "STOP", OLED_6X8);
						  OLED_Update();}
			
			Mode_3_Cur_State = STATE_BALANCE_OFF;
			Head_PID_control_enable = 0;
			//强制停止（电机）运行
			motor_SetPWM(1, 0);
			motor_SetPWM(2, 0);
			DifPWM  = 0;
			
			Run_Flag = 0;
		}
		
		
		/*======================================================*/
		/*[路径处理]*********************************************/
		/*======================================================*/
		if ((Time_Count2 >= 3 && Track_Scan_flag == 0)||
			Time_Count2 >= 8 && Track_Scan_flag == 1)
		{
			Track_Scan_flag += 1;
			
			float Error = Track_Sensor_Get_Error();
			Error_filtered = 0.9 * Error + 0.1 * Error_filtered;
		}
		if (Track_Scan_flag == 2 && Mode_3_Turns_Count < 4)
		{
			Track_Scan_flag = 0;// 循迹周期相关的变量重置
			
//			OLED_Printf(36, 24, OLED_6X8, "%2.1f", Error);			
			OLED_Printf(36, 32, OLED_6X8, "%2.1f", Yaw_Result);
			switch(Track_Sensor_State)//是否在线
			{
				// 在线
				case TRACK_STATE_ON_LINE:
				{
//					OLED_ShowString(36, 8 , "ON ", OLED_6X8);
					
					// ========== 状态切换判定（受冷却影响）==========
					if (Delay_Timer_2 == 0)
					{
						if (Mode_3_Cur_State == STATE_A_TO_C)
						{
							// 到达C
							Speed_PID.Target = 35;
							Mode_3_Cur_State = STATE_C_TO_B;
							Head_PID_control_enable = 0;
							Turn_Encoder_Accum = 0;// 弯道行驶距离 重置
							Turn_State_flag = 1;// 弯道行驶距离累积 开启
							Straight_State_flag = 0; // 直线行驶距离累积 关闭
							Delay_Timer_1 = 50;// 声光触发
							Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
							
							OLED_ShowString(36, 16, "C->B", OLED_6X8);
//							bluetooth_ch04_printf("C\r\n");
						}
						else if (Mode_3_Cur_State == STATE_B_TO_D)
						{
							// 到达D
							Speed_PID.Target = 35;
							Mode_3_Cur_State = STATE_D_TO_A;
							Head_PID_control_enable = 0;							
							Turn_Encoder_Accum = 0;// 弯~ 重置
							Turn_State_flag = 1;// 弯~ 开启
							Straight_State_flag = 0;// 直~ 关闭
							Delay_Timer_1 = 50;// 声光触发
							Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
							
							OLED_ShowString(36, 16, "D->A", OLED_6X8);
//							bluetooth_ch04_printf("D\r\n");
						}
					}
					
					// ========== 巡线执行（不受冷却影响）==========
					if (Mode_3_Cur_State == STATE_PREP ||
						Mode_3_Cur_State == STATE_C_TO_B ||
						Mode_3_Cur_State == STATE_D_TO_A)
					{
						Speed_PID.Target = 35;
						Track_PID.Actual = Error_filtered;
						PID_Update(&Track_PID);
						Track_Out_filtered = 0.7 * Track_PID.Out + 0.3 * Track_Out_filtered;
						Turn__PID.Target = Track_Out_filtered;
					}
					
					break;
				}
				// 掉线
				case TRACK_STATE_OFF_LINE:
				{
//					OLED_ShowString(36, 8 , "OFF", OLED_6X8);				
				
					// ========== 状态切换判定（受冷却影响）==========
					if (Delay_Timer_2 == 0)
					{
						if (Mode_3_Cur_State == STATE_PREP)
						{
							// 到达A（如果是放在引导线上发车）
							Speed_PID.Target = 0;// 准备原地转向（转向逻辑交给下面的子状态）
							Mode_3_Cur_State = STATE_A_TO_C;		
							Cur_Inner_State = INNER_STATE_TURN_RIGHT;
							Yaw_Target = Yaw_Result;// 标定发车Yaw角，这将作为后续行驶的基准
							Delay_Timer_1 = 50;// 声光触发
							Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
							
							OLED_ShowString(36, 16, "A->C", OLED_6X8);
//							bluetooth_ch04_printf("A\r\n");
						}						
						else if (Mode_3_Cur_State == STATE_C_TO_B && 
									(Yaw_Target + 177 <= Yaw_Result && Yaw_Result <= Yaw_Target + 183) &&
									Turn_Encoder_Accum >= 5400)
						{
							// 到达B
							Mode_3_Cur_State = STATE_B_TO_D;
							Cur_Inner_State = INNER_STATE_TURN_RIGHT;
							Head_PID_control_enable = 0;
							Turn_Encoder_Accum = 0;// 弯~ 重置
							Turn_State_flag = 0;// 弯~ 关闭
							Delay_Timer_1 = 50;// 声光触发
							Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
							
							OLED_ShowString(36, 16, "B->D", OLED_6X8);
//							bluetooth_ch04_printf("B\r\n");	
						}
						else if (Mode_3_Cur_State == STATE_D_TO_A &&
									(Yaw_Target - 3 <= Yaw_Result && Yaw_Result <= Yaw_Target + 3) &&
									Turn_Encoder_Accum >= 5400)
						{
							// 到达A
							Mode_3_Cur_State = STATE_A_TO_C;
							Cur_Inner_State = INNER_STATE_TURN_RIGHT;
							Head_PID_control_enable = 0;
							Mode_3_Turns_Count ++;// 圈数累加
							Turn_Encoder_Accum = 0;// 弯~ 重置
							Turn_State_flag = 0;// 弯~ 关闭
							Delay_Timer_1 = 50;// 声光触发
							Delay_Timer_2 = TRACK_SWITCH_COOLDOWN;// 点位切换冷却
							
							OLED_ShowString(36, 16, "A->C", OLED_6X8);
//							bluetooth_ch04_printf("A\r\n");
						}
					}
					
					// ========== 平滑处理（不受冷却影响）==========
					// 到点的判定在前面执行，理论上这里不会影响到点判定
					// 在转弯状态掉线时，保持循迹环输出的平滑衰减
					if (Mode_3_Cur_State == STATE_C_TO_B || 
						Mode_3_Cur_State == STATE_D_TO_A)
					{
						Track_Out_filtered = 0.8 * Track_Out_filtered + 0.2 * 0;
						Turn__PID.Target = Track_Out_filtered;
					}
					
					// ========== 航向角控制（不受冷却影响）==========
					if (Mode_3_Cur_State == STATE_A_TO_C && Mode_3_Turns_Count < 4)
					{
						switch(Cur_Inner_State)
						{
							case INNER_STATE_TURN_RIGHT:
							{
								// 右转固定角度（相对子状态开始时）
								Speed_PID.Target = 0; // 原地转向
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target - 50;								
								
								// 检查是否完成转向
								if (fabs(Yaw_Result - Head__PID.Target) < 2.0f)
								{
									Cur_Inner_State = INNER_STATE_GO_STRAIGHT_1;
									Straight_Encoder_Accum = 0;// 直~ 重置
									Straight_State_flag = 1;// 直~ 开启
								}
								break;
							}
							case INNER_STATE_GO_STRAIGHT_1:
							{
								// 直线行驶
								Speed_PID.Target = 35;
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target - 50;
																
								// 检查是否到达目标距离
								if (Straight_Encoder_Accum > 5000)
								{
									Cur_Inner_State = INNER_STATE_TURN_LEFT;// 准备左转
									Straight_Encoder_Accum = 0;// 直~ 重置
									Straight_State_flag = 0;// 直~ 关闭
								}
								break;
							}
							case INNER_STATE_TURN_LEFT:
							{
								// 左转固定角度
								Speed_PID.Target = 0; // 原地转向
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target;								
								
								// 检查是否完成转向
								if (fabs(Yaw_Result - Head__PID.Target) < 2.0f)
								{
									// 转向完成，等待寻到线
									Cur_Inner_State = INNER_STATE_GO_STRAIGHT_2;
									
								}
								break;
							}
							case INNER_STATE_GO_STRAIGHT_2:
							{	
								// 直线行驶直到到达C
								Speed_PID.Target = 35;
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target;
								
								break;
							}
						}
					}
					else if (Mode_3_Cur_State == STATE_B_TO_D && Mode_3_Turns_Count < 4)
					{
						switch(Cur_Inner_State)
						{
							case INNER_STATE_TURN_RIGHT:
							{
								// 右转固定角度（相对子状态开始时）
								Speed_PID.Target = 0; // 原地转向
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target + 230;// 180 + 50
															
								// 检查是否完成转向
								if (fabs(Yaw_Result - Head__PID.Target) < 2.0f)
								{
									Cur_Inner_State = INNER_STATE_GO_STRAIGHT_1;
									Straight_Encoder_Accum = 0;// 直~ 重置
									Straight_State_flag = 1;// 直~ 开启
								}
								break;
							}
							case INNER_STATE_GO_STRAIGHT_1:
							{
								// 直线行驶
								Speed_PID.Target = 35;
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target + 230;// 180 + 50
								
								// 检查是否到达目标距离
								if (Straight_Encoder_Accum > 5000)
								{
									Cur_Inner_State = INNER_STATE_TURN_LEFT;
									Straight_Encoder_Accum = 0;// 直~ 重置
									Straight_State_flag = 0;// 直~ 关闭
								}
								break;
							}
							case INNER_STATE_TURN_LEFT:
							{
								// 左转固定角度
								Speed_PID.Target = 0; // 原地转向
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target + 180;								
								
								// 检查是否完成转向
								if (fabs(Yaw_Result - Head__PID.Target) < 2.0f)
								{
									// 转向完成，等待寻到线
									Cur_Inner_State = INNER_STATE_GO_STRAIGHT_2;								
								}
								break;
							}
							case INNER_STATE_GO_STRAIGHT_2:
							{	
								// 直线行驶直到到达C
								Speed_PID.Target = 35;
								Head_PID_control_enable = 1;
								Head__PID.Target = Yaw_Target + 180;

								break;
							}
						}
					}				
					break;
				}
			}
			if (Mode_3_Turns_Count >= 4)
			{
				// 跑完四圈
				Speed_PID.Target = 0;
				Mode_3_Cur_State = STATE_STOP;
				Turn_State_flag = 0;										
				Turn__PID.Target = 0;
				Head_PID_control_enable = 0;
				Delay_Timer_1 = 50;// 声光触发
				
				OLED_ShowString(36, 16, "STOP", OLED_6X8);
//				bluetooth_ch04_printf("P\r\n");
			
			}
			OLED_Update();
		}
		/*======================================================*/
		/*********************************************[路径处理]*/
		/*======================================================*/
			
		
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
		
		
		/* 外圈调控周期*/
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
			// 直线行驶距离累积
			if (Straight_State_flag){Straight_Encoder_Accum += Encoder_Left + Encoder_Right;}
			
			// 实际速度换算
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;	// 实际平均速度
			DifSpeed = LeftSpeed - RightSpeed;			// 实际差分速度
			
			if (Run_Flag)
			{
				/* 航向角PID介入（挪用的时候注意航向角环输出取反给转向环）*/
				if (Head_PID_control_enable)
				{
					Head__PID.Actual = Yaw_Result;
					PID_Update(&Head__PID);
					Turn__PID.Target = - Head__PID.Out;
				}
				
				/* 转向环+速度环PID计算*/
				PID_Calc_Speed_And_Turn();
			}
		}

		
		/* 内圈调控周期*/
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
