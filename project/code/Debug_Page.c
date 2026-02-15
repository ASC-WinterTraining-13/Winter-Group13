#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "param_config.h"
#include "param_storage.h"
#include "OLED.h"

#include "Track_Sensor.h"
#include "Encoder.h"
#include "mpu6050_Analysis.h"
#include "zf_device_bluetooth_ch04.h"
#include "BuzzerAndLED.h"
#include "motor.h"

/*******************************************************************************************************************/
/*[S] 界面样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]调试选项界面
void Debug_Page_Menu_UI(uint8_t Page)
{
	switch(Page)
	{
		// 第一页
		case 1:
			OLED_ShowString(8 , 0 , "Debug", OLED_6X8);
			OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
			OLED_ShowString(10, 16, "Track_Sensor", OLED_8X16);
			OLED_ShowString(10, 32, "Encoder", OLED_8X16);
			OLED_ShowString(10, 48, "MPU6050_Calib", OLED_8X16);
		
			break;
		
		// 第二页
		case 2:
			OLED_ShowString(10, 0 , "Bluetooth", OLED_8X16);
			OLED_ShowString(10, 16, "BUZ&LED", OLED_8X16);
			OLED_ShowString(10, 32, "PWM", OLED_8X16);
		
			break;
	}
}



// [三级界面]循迹模块调试
void Debug_Track_Sensor_UI(void)
{
	OLED_ShowString(8 , 0 , "Track_Sensor", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(32, 16, "L1 L2 R2 R1", OLED_8X16);
	OLED_ShowString(0 , 32, "cur: ?  ?  ?  ?", OLED_8X16);
	OLED_ShowString(0 , 48, "Error:???", OLED_8X16);
}

// [三级界面]编码器调试
void Debug_Encoder_UI(void)
{
	OLED_ShowString(8 , 0 , "Encoder", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(0 , 16, "Sp_L:", OLED_6X8);
	OLED_ShowString(0 , 24, "Sp_R:", OLED_6X8);
	OLED_ShowString(0 , 32, "Sum_L:", OLED_6X8);
	OLED_ShowString(0 , 40, "Sum_R:", OLED_6X8);
}

// [三级界面]MPU6050校准按钮
void Calib_Button_UI(uint8_t calib_state)
{
	OLED_ShowString(8 , 0 , "MPU6050_Calib", OLED_6X8);
	OLED_ShowString(0 , 8, "=====================", OLED_6X8);
	
	switch (calib_state)
	{
		case 0:
			OLED_ShowString(0, 16, "未校准", OLED_8X16);
			break;
		
//		case 1:
//			OLED_ShowString(0, 16, "校准中", OLED_8X16);
//			break;
		
		case 2:
			OLED_ShowString(0, 16, "已校准", OLED_8X16);
			break;	
	}
}

// [三级界面]蓝牙模块调试
void Debug_Bluetooth_UI(void)
{
	OLED_ShowString(8 , 0 , "Bluetooth", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(0 , 16, "TX:", OLED_8X16);
	OLED_ShowString(0 , 32, "RX:", OLED_8X16);
}

// [三级界面]声光模块调试
void Debug_BUZandLED_UI(void)
{
	OLED_ShowString(8 , 0 , "BUZ+LED", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(0 , 16, "BUZ:", OLED_8X16);
	OLED_ShowString(0 , 32, "LED:", OLED_8X16);
}

// [三级界面]电机驱动调试
void Debug_PWM_UI(void)
{
	OLED_ShowString(8 , 0 , "PWM", OLED_6X8);
	OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
	OLED_ShowString(10, 16, "PWM_L:", OLED_8X16);
	OLED_ShowString(10, 32, "PWM_R:", OLED_8X16);
}
/*******************************************************************************************************************/
/*[E] 界面样式 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 交互界面 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int Debug_Track_Sensor	(void);
int Debug_Encoder		(void);
int Calib_Button_Page	(void);
int Debug_Bluetooth		(void);
int Debug_BUZandLED		(void);
int Debug_PWM			(void);

// [二级界面]调试选项界面
int Debug_Page_Menu(void)
{
	// 调试界面光标 标志位
    uint8_t Debug_Page_flag = 1;
	
	Debug_Page_Menu_UI(1);
	OLED_ShowString(0 , 16, ">", OLED_8X16);
	OLED_Update();
	
	while(1)
	{
		// 存储确认键被按下时Debug_Page_flag的值的临时变量，默认为无效值0
		uint8_t Debug_Page_flag_temp = 0;
		
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;
		
		/* 按键解析*/
		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
		{
			key_clear_state(KEY_UP);
			key_pressed  = 1;
			Debug_Page_flag --;
			if (Debug_Page_flag < 1)Debug_Page_flag = 6;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{
			key_clear_state(KEY_DOWN);
			key_pressed = 1;
			Debug_Page_flag ++;
			if (Debug_Page_flag > 6)Debug_Page_flag = 1;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
		{
			key_clear_state(KEY_CONFIRM);
			Debug_Page_flag_temp = Debug_Page_flag;
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			
			return 0;
		}
		
		/* 模式跳转*/
		if (Debug_Page_flag_temp == 1)
		{
			OLED_Clear();
			Debug_Track_Sensor();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(1);
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
		}
		else if (Debug_Page_flag_temp == 2)
		{
			OLED_Clear();
			Debug_Encoder();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(1);
            OLED_ShowString(0, 32, ">", OLED_8X16);
			OLED_Update();
		}
		else if (Debug_Page_flag_temp == 3)
		{
			OLED_Clear();
			Calib_Button_Page();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(1);
            OLED_ShowString(0, 48, ">", OLED_8X16);
			OLED_Update();
		}
		else if (Debug_Page_flag_temp == 4)
		{
			OLED_Clear();
			Debug_Bluetooth();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(2);
            OLED_ShowString(0, 0, ">", OLED_8X16);
			OLED_Update();
		}
		else if (Debug_Page_flag_temp == 5)
		{
			OLED_Clear();
			Debug_BUZandLED();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(2);
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
		}
		else if (Debug_Page_flag_temp == 6)
		{
			OLED_Clear();
			Debug_PWM();
			
			// 从模式返回后重新显示菜单
			OLED_Clear();						
            Debug_Page_Menu_UI(2);
            OLED_ShowString(0, 32, ">", OLED_8X16);
			OLED_Update();
		}
		
		
		/* 菜单显示更新*/
		// 判断是否需要更新
		if (key_pressed)
		{			
			switch(Debug_Page_flag)
			{
				case 1:
					OLED_Clear();						
					Debug_Page_Menu_UI(1);
					OLED_ShowString(0, 16, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 2:
					OLED_Clear();						
					Debug_Page_Menu_UI(1);
					OLED_ShowString(0, 32, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 3:
					OLED_Clear();						
					Debug_Page_Menu_UI(1);
					OLED_ShowString(0, 48, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 4:
					OLED_Clear();						
					Debug_Page_Menu_UI(2);
					OLED_ShowString(0, 0 , ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 5:
					OLED_Clear();						
					Debug_Page_Menu_UI(2);
					OLED_ShowString(0, 16, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 6:
					OLED_Clear();						
					Debug_Page_Menu_UI(2);
					OLED_ShowString(0, 32, ">", OLED_8X16);
					OLED_Update();
				
					break;
			}
		}
	}
}


// [三级界面]循迹模块调试
int Debug_Track_Sensor(void)
{
	Debug_Track_Sensor_UI();
	OLED_Update();
	
	uint8_t sensor_data[4] = {0};
	float error = 0.0f;
	
	
	while(1)
	{
		/* 按键解析*/
//		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
//		{
//			key_clear_state(KEY_UP);
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
//		{
//			key_clear_state(KEY_DOWN);	
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
//		{
//			key_clear_state(KEY_CONFIRM);
//		}
//		else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			
			// 返回上一级菜单
			return 0;
		}
		
		
		Track_Sensor_Get_All_Status(sensor_data);
		error = Track_Sensor_Get_Error();
		OLED_Printf(40, 32, OLED_8X16, "%d  %d  %d  %d", sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]);
		OLED_Printf(48, 48, OLED_8X16, "%3.2f  ", error);
		OLED_Update();	
	}
}

// [三级界面]编码器调试
int Debug_Encoder(void)
{
	Debug_Encoder_UI();
	OLED_Update();
	
	float Sum_LeftSpeed = 0.0f;
	float Sum_RightSpeed = 0.0f;
	
	
	while(1)
	{
		/* 按键解析*/
//		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
//		{
//			key_clear_state(KEY_UP);
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
//		{
//			key_clear_state(KEY_DOWN);	
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
//		{
//			key_clear_state(KEY_CONFIRM);
//		}
//		else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			
			// 返回上一级菜单
			return 0;
		}
		
				/* 速度计算*/
		if (Time_Count2 > 20)// 20 * 5 ms调控周期
		{
			Time_Count2 = 0;
			
			LeftSpeed  = Get_Encoder1() * 0.6f + Pre_LeftSpeed  * 0.4f;
			RightSpeed = Get_Encoder2() * 0.6f + Pre_RightSpeed * 0.4f;
			Pre_LeftSpeed = LeftSpeed;
			Pre_RightSpeed = RightSpeed;
			
			Sum_LeftSpeed += LeftSpeed;
			Sum_RightSpeed += RightSpeed;

			OLED_Printf(30, 16, OLED_6X8, "%2.2f  ", LeftSpeed);
			OLED_Printf(30, 24, OLED_6X8, "%2.2f  ", RightSpeed);
			OLED_Printf(30, 32, OLED_6X8, "%4.2f  ", Sum_LeftSpeed);
			OLED_Printf(30, 40, OLED_6X8, "%4.2f  ", Sum_RightSpeed);
			
			OLED_Update();
		}
	}
}

// [三级界面]MPU6050校准按钮
int Calib_Button_Page(void)
{
	// 如果已校准
	if (MPU6050_Calibration_Check() == 2)
	{
		Calib_Button_UI(2);
		OLED_Update();
	}    
	// 那就是未校准了
	else 
	{
		Calib_Button_UI(0);
		OLED_Update();
	}
	
	
	while(1)
	{
		/* 按键解析*/
		//		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
//		{
//			key_clear_state(KEY_UP);
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
//		{
//			key_clear_state(KEY_DOWN);	
//		}
//		else
		if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))// 确认键
        {
            key_clear_state(KEY_CONFIRM);
			
			OLED_ShowString(0, 16, "校准中", OLED_8X16);
			OLED_Update();
			
			/* mpu6050零飘校准逻辑(此时请保持静止)*/
			MPU6050_Calibration_Start();			
			// 半阻塞式零飘校准
			while(1)  
			{
				if (MPU6050_Calibration_Check() == 2)  // 零飘校准完成
				{
					OLED_ShowString(0, 16, "已校准", OLED_8X16);
					OLED_Update();
					break;  // 跳出零飘校准循环，往下执行
				}				
				// 可以考虑在这里操作OLED，但也请注意时间占用
				
				// 强制零飘校准退出
				if(KEY_SHORT_PRESS == key_get_state(KEY_BACK))
				{
					key_clear_state(KEY_BACK);
					
					OLED_ShowString(0, 16, "已取消", OLED_8X16);
					OLED_Update();
					break;  // 退出零飘校准模式
				}       
			}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))// 返回键
        {
            key_clear_state(KEY_BACK);

			// 返回上一级菜单
            return 0;
        }
	}
}

// [三级界面]蓝牙模块调试
int Debug_Bluetooth(void)
{
	Debug_Bluetooth_UI();
	OLED_Update();
	
	// ========== 蓝牙接收缓冲区 ==========
	static uint8 bluetooth_rx_buffer[100];
	static uint32 bluetooth_rx_length = 0;
	
	uint8_t tset_state = 0;
	
	while(1)
	{
		/* 按键解析*/
//		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
//		{
//			key_clear_state(KEY_UP);
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
//		{
//			key_clear_state(KEY_DOWN);	
//		}
//		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
//		{
//			key_clear_state(KEY_CONFIRM);
//		}
//		else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			
			// 返回上一级菜单
			return 0;
		}
		
		
		// 检查是否接收到完整数据包
		if(bluetooth_ch04_get_rx_flag() == 1)
		{
			// 读取接收缓冲区中的数据
			bluetooth_rx_length = bluetooth_ch04_read_buffer(bluetooth_rx_buffer, sizeof(bluetooth_rx_buffer) - 1);
			
			if(bluetooth_rx_length > 0)
			{
				// 确保字符串以 null 结尾
				bluetooth_rx_buffer[bluetooth_rx_length] = '\0';
				
//				// 调试输出：显示原始接收数据
//				printf("BT RX: [");
//				for(uint32 i = 0; i < bluetooth_rx_length; i++)
//				{
//					printf("%c", bluetooth_rx_buffer[i]);
//				}
//				printf("]\r\n");
//				OLED_Printf(24, 16, OLED_8X16, "%s", );
				
				OLED_Printf(24, 32, OLED_8X16, "%s", bluetooth_rx_buffer);
				OLED_Update();
			}
			
			// 清除接收标志位（备用，bluetooth_ch04_read_buffer已清除）
			bluetooth_ch04_clear_rx_flag();
		}
		
		
		if (tset_state == 0 || Time_Count2 > 1000 && tset_state == 2)
		{
			tset_state = 1;
			bluetooth_ch04_printf("TEST\r\n");
			OLED_Printf(24, 16, OLED_8X16, "TSET");
			OLED_Update();
		}
		else if (Time_Count2 > 2000 && tset_state == 1)
		{
			Time_Count2 = 0;
			tset_state = 2;
			bluetooth_ch04_printf("test\r\n");
			OLED_Printf(24, 16, OLED_8X16, "test");
			OLED_Update();
		}
	}
}

// [三级界面]声光模块调试
int Debug_BUZandLED(void)
{
	Debug_BUZandLED_UI();
	OLED_ShowString(32, 16, "OFF", OLED_8X16);
	OLED_ShowString(32, 32, "OFF", OLED_8X16);
	OLED_Update();
	
    uint8_t BUZ_flag = 0;
	uint8_t LED_flag = 0;
	
	while(1)
	{
		// 存储确认键被按下时Debug_BUZandLED_flag的值的临时变量，默认为无效值0
		uint8_t Debug_BUZandLED_flag_temp = 0;
		
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;
		
		/* 按键解析*/
		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
		{
			key_clear_state(KEY_UP);
			BUZ_flag = 1 - BUZ_flag;
			if (BUZ_flag){BUZ_SET(1);OLED_ShowString(32, 16, "ON ", OLED_8X16);OLED_Update();}
			else {BUZ_SET(0);OLED_ShowString(32, 16, "OFF", OLED_8X16);OLED_Update();}
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{
			key_clear_state(KEY_DOWN);	
			LED_flag = 1- LED_flag;
			if (LED_flag){LED_SET(1);OLED_ShowString(32, 32, "ON ", OLED_8X16);OLED_Update();}
			else {LED_SET(0);OLED_ShowString(32, 32, "OFF", OLED_8X16);OLED_Update();}
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
				 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_CONFIRM);
			key_clear_state(KEY_BACK);
			BUZ_SET(0);
			LED_SET(0);
			return 0;  // 退出修改模式
		}
	}
}

// [三级界面]电机驱动调试
int Debug_PWM(void)
{
	Debug_PWM_UI();
	OLED_ShowString(0, 16, ">", OLED_8X16);
	OLED_Printf(58, 16, OLED_8X16, "%d    ", LeftPWM);
	OLED_Printf(58, 32, OLED_8X16, "%d    ", RightPWM);
	OLED_Update();
	
	// 电机驱动调试界面光标 标志位
    uint8_t Debug_PWM_flag = 1;
	
	
	while(1)
	{
		// 存储确认键被按下时Debug_PWM_flag的值的临时变量，默认为无效值0
		uint8_t Debug_PWM_flag_temp = 0;
		
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;
		
		/* 按键解析*/
		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
		{
			Debug_PWM_flag --;
			if (Debug_PWM_flag < 1){Debug_PWM_flag = 2;}
			key_clear_state(KEY_UP);
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{
			Debug_PWM_flag ++;
			if (Debug_PWM_flag > 2){Debug_PWM_flag = 1;}
			key_clear_state(KEY_DOWN);	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
		{
			Debug_PWM_flag_temp = Debug_PWM_flag;
			key_clear_state(KEY_CONFIRM);
		}
		else 
		if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			
			// 返回上一级菜单
			return 0;
		}
		
		
		/* 功能跳转*/
		if (Debug_PWM_flag_temp == 1)
		{
			OLED_ShowString(0, 16, "=", OLED_8X16);
			OLED_Update();
			
			while(1)
			{
				/* 按键解析*/
				if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
				{
					key_clear_state(KEY_UP);
					LeftPWM += 100;
					if (LeftPWM > 10000)LeftPWM = 10000;
					motor_SetPWM(1, LeftPWM);
					OLED_Printf(58, 16, OLED_8X16, "%d    ", LeftPWM);
					OLED_Update();
				}
				else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
				{
					key_clear_state(KEY_DOWN);
					LeftPWM -= 100;
					if (LeftPWM > 10000)LeftPWM = 10000;
					motor_SetPWM(1, LeftPWM);
					OLED_Printf(58, 16, OLED_8X16, "%d    ", LeftPWM);
					OLED_Update();

				}
				else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
						 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
				{
					key_clear_state(KEY_CONFIRM);
					key_clear_state(KEY_BACK);
					
					break;  // 退出修改模式
				}
			}
		}
		else if (Debug_PWM_flag_temp == 2)
		{
			OLED_ShowString(0, 32, "=", OLED_8X16);
			OLED_Update();
			
			while(1)
			{
				/* 按键解析*/
				if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
				{
					key_clear_state(KEY_UP);
					RightPWM += 100;
					if (RightPWM > 10000)RightPWM = 10000;
					motor_SetPWM(1, RightPWM);
					OLED_Printf(58, 32, OLED_8X16, "%d    ", RightPWM);
					OLED_Update();
				}
				else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
				{
					key_clear_state(KEY_DOWN);
					RightPWM -= 100;
					if (RightPWM > 10000)RightPWM = 10000;
					motor_SetPWM(1, RightPWM);
					OLED_Printf(58, 32, OLED_8X16, "%d    ", RightPWM);
					OLED_Update();

				}
				else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
						 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
				{
					key_clear_state(KEY_CONFIRM);
					key_clear_state(KEY_BACK);
					
					break;  // 退出修改模式
				}
			}
		}
		
		
		/* 显示更新*/
		switch(Debug_PWM_flag)
		{
			case 1:
				OLED_ShowString(0, 16, ">", OLED_8X16);
				OLED_ShowString(0, 32, " ", OLED_8X16);
				OLED_Update();
				
				break;
			
			case 2:
				OLED_ShowString(0, 16, " ", OLED_8X16);
				OLED_ShowString(0, 32, ">", OLED_8X16);
				OLED_Update();
				
				break;
		}
	}
}
/*******************************************************************************************************************/
/*[E] 交互界面 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
