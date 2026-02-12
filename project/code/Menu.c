#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "Mode_1.h"
#include "Mode_2.h"
#include "Mode_3.h"
#include "Mode_4.h"
#include "Mode_5.h"
#include "Core_Param.h"
#include "SandBox_Page.h"
#include "Calib_Button.h"

#include "param_config.h"
#include "zf_device_bluetooth_ch04.h"
#include "Encoder.h"
#include "motor.h"
#include "BuzzerAndLED.h"
#include "OLED.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外设初始化
// 使用示例     Peripheral_Init();
// 备注信息     只在"main.c"中调用一次
//-------------------------------------------------------------------------------------------------------------------

void Peripheral_Init(void)
{
	/* （七针脚）OLED初始化*/
    OLED_Init();
	
	/* 按键初始化（10ms扫描周期）*/
    key_init(10);
	
	/* mpu6050初始化*/
	mpu6050_init();
	
	/* 编码器初始化*/
	Encoder_Init();
	
	/* CH-04蓝牙模块初始化*/
	bluetooth_ch04_init();
	
	/* 电机初始化*/
	motor_Init();
	
	/* 声光模块初始化*/
	BuzzerAndLED_Init();
}


/*******************************************************************************************************************/
/*[S] 菜单样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [一级界面]主菜单界面
void Menu_UI(uint8_t Page)
{
	switch(Page)
	{		
		// 第一页
		case 1:
		{
			OLED_ShowString(8 , 0 , "Menu", OLED_6X8);
			OLED_ShowString(0 , 8 , "=====================", OLED_6X8);
			OLED_ShowString(10, 16, "Mode_1", OLED_8X16);
			OLED_ShowString(10, 32, "Mode_2", OLED_8X16);
			OLED_ShowString(10, 48, "Mode_3", OLED_8X16);
			
			break;
		}
		
		// 第二页
		case 2:
		{

			OLED_ShowString(10, 0 , "Mode_4", OLED_8X16);
			OLED_ShowString(10, 16, "Mode_5", OLED_8X16);
			OLED_ShowString(10, 32, "Core_Param", OLED_8X16);
			OLED_ShowString(10, 48, "SandBox_Page", OLED_8X16);
			
			break;
		}
		// 第三页
		case 3:
		{
			
			OLED_ShowString(10, 0 , "Calib_Button", OLED_8X16);
			
			break;
		}
	}
}

/*******************************************************************************************************************/
/*[E] 菜单样式 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 交互界面 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 菜单选项标志位
uint8_t menuflag = 1;

void Menu_Show(void)
{
	// 显示菜单
	Menu_UI(1);
	OLED_ShowString(0, 16, ">", OLED_8X16);
	OLED_Update();
		
	while(1)
	{
		// 存储确认键被按下时menuflag的值的临时变量，默认为无效值0
		uint8_t menuflag_temp = 0;
		
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;
				
		/* 按键解析*/
		if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
		{
			key_clear_state(KEY_UP);
			key_pressed  = 1;
			menuflag --;
			if (menuflag < 1)menuflag = 8;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{
			key_clear_state(KEY_DOWN);
			key_pressed = 1;
			menuflag ++;
			if (menuflag > 8)menuflag = 1;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
		{
			key_clear_state(KEY_CONFIRM);
			menuflag_temp = menuflag;
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			// 照理不应该有功能
		}
		
		/* 模式跳转*/
		// 模式1
		if (menuflag_temp == 1)
		{
			OLED_Clear();
			Mode_1_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();						
            Menu_UI(1);
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
		}
		// 模式2
		else if (menuflag_temp == 2)
		{
			OLED_Clear();
			Mode_2_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(1);
            OLED_ShowString(0, 32, ">", OLED_8X16);
			OLED_Update();
		}
		// 模式3
		else if (menuflag_temp == 3)
		{
			OLED_Clear();
			Mode_3_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(1);
            OLED_ShowString(0, 48, ">", OLED_8X16);
			OLED_Update();
		}
		// 模式4
		else if (menuflag_temp == 4)
		{
			OLED_Clear();
			Mode_4_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(2);
            OLED_ShowString(0, 0 , ">", OLED_8X16);
			OLED_Update();
		}
		// 模式5
		else if (menuflag_temp == 5)
		{
			OLED_Clear();
			Mode_5_Menu();
		
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(2);
            OLED_ShowString(0, 16, ">", OLED_8X16);
			OLED_Update();
		}	
		// 核心参数设置界面
		else if (menuflag_temp == 6)
		{
			OLED_Clear();
			Core_Param_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(2);
            OLED_ShowString(0, 32, ">", OLED_8X16);
			OLED_Update();
		}
		// 空白调试界面
		else if (menuflag_temp == 7)
		{
			OLED_Clear();
			SandBox_Page();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(2);
            OLED_ShowString(0, 48, ">", OLED_8X16);
			OLED_Update();
		}
		// 空白调试界面
		else if (menuflag_temp == 8)
		{
			OLED_Clear();
			Calib_Button_Menu();
			
			// 从模式返回后
			Run_Flag = 0;
			// 重新显示菜单
			OLED_Clear();			
            Menu_UI(3);
            OLED_ShowString(0, 0 , ">", OLED_8X16);
			OLED_Update();
		}		

		
		/* 菜单显示更新*/
		// 判断是否需要更新
		if (key_pressed)
		{			
			switch(menuflag)
			{
				case 1:
					OLED_Clear();						
					Menu_UI(1);
					OLED_ShowString(0, 16, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 2:
					OLED_Clear();						
					Menu_UI(1);
					OLED_ShowString(0, 32, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 3:
					OLED_Clear();						
					Menu_UI(1);
					OLED_ShowString(0, 48, ">", OLED_8X16);
					OLED_Update();
					
					break;
				
				case 4:
					OLED_Clear();						
					Menu_UI(2);
					OLED_ShowString(0, 0 , ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 5:
					OLED_Clear();						
					Menu_UI(2);
					OLED_ShowString(0, 16, ">", OLED_8X16);
					OLED_Update();
				
					break;
				case 6:
					OLED_Clear();						
					Menu_UI(2);
					OLED_ShowString(0, 32, ">", OLED_8X16);
					OLED_Update();
				
					break;
				case 7:
					OLED_Clear();						
					Menu_UI(2);
					OLED_ShowString(0, 48, ">", OLED_8X16);
					OLED_Update();
				
					break;
				
				case 8:
					OLED_Clear();						
					Menu_UI(3);
					OLED_ShowString(0, 0 , ">", OLED_8X16);
					OLED_Update();
				
					break;
			}
		}
	
	}
}

/*******************************************************************************************************************/
/*[E] 交互界面 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

