#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "Mode_1.h"
#include "Mode_2.h"
#include "Mode_3.h"
#include "Mode_4.h"
#include "Mode_5.h"
#include "zf_device_bluetooth_ch04.h"
#include "Encoder.h"

/*--------------------[S] 外设初始化 [S]--------------------*/

void Peripheral_Init(void)
{
	/* （七针脚）OLED初始化*/
    oled_init();
    // 可以使用的字体为6X8和8X16,点阵坐标等效是基于6X8标定的
    oled_set_font(OLED_8X16_FONT);    
	oled_clear();
	
	/* 按键初始化（10ms扫描周期）*/
    key_init(10);
	
	/* mpu6050初始化*/
	mpu6050_init();
	
	/* 编码器初始化*/
	Encoder_Init();
	
	/*CH-04蓝牙模块初始化*/
	bluetooth_ch04_init();
}
/*--------------------[E] 外设初始化 [E]--------------------*/


/*--------------------[S] 菜单样式 [S]--------------------*/

void Menu_UI(uint8_t Page)
{
	switch(Page){
		
		// 第一页
		case 1:
		{
			oled_show_string(0, 0, "MENU");
			oled_show_string(0, 2, "===");
			oled_show_string(2, 4, " Mode_1");
			oled_show_string(2, 6, " Mode_2");

			break;
		}
		
		// 第二页
		case 2:
		{
			oled_show_string(2, 0, " Mode_3");
			oled_show_string(2, 2, " Mode_4");
			oled_show_string(2, 4, " Mode_5");
			
			break;
		}
	}
}
/*--------------------[E] 菜单样式 [E]--------------------*/


/*--------------------[S] 交互界面 [S]--------------------*/

// 菜单选项标志位
uint8_t menuflag = 1;

void Menu_Show(void)
{
	// 显示菜单
	Menu_UI(1);
	oled_show_string(0, 4, ">");
		
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
			if (menuflag < 1)menuflag = 5;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
		{
			key_clear_state(KEY_DOWN);
			key_pressed = 1;
			menuflag ++;
			if (menuflag > 5)menuflag = 1;	
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
		{
			key_clear_state(KEY_CONFIRM);
			menuflag_temp = menuflag;
		}
		else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
		{
			key_clear_state(KEY_BACK);
			//
		}
		
		/* 模式跳转*/
		// 模式1
		if (menuflag_temp == 1)
		{
			oled_clear();
			Mode_1_Menu();
			// 返回后重新显示菜单
			oled_set_font(OLED_8X16_FONT);  
            Menu_UI(1);
            oled_show_string(0, 4, ">");
		}
		// 模式2
		else if (menuflag_temp == 2)
		{
			oled_clear();
			Mode_2_Menu();
			// 返回后重新显示菜单
			oled_set_font(OLED_8X16_FONT);  
            Menu_UI(1);
            oled_show_string(0, 6, ">");
		}
		// 模式3
		else if (menuflag_temp == 3)
		{
			oled_clear();
			Mode_3_Menu();
			// 返回后重新显示菜单
			oled_set_font(OLED_8X16_FONT);  
            Menu_UI(2);
            oled_show_string(0, 0, ">");
		}
		// 模式4
		else if (menuflag_temp == 4)
		{
			oled_clear();
			Mode_4_Menu();
			// 返回后重新显示菜单
			oled_set_font(OLED_8X16_FONT);  
            Menu_UI(2);
            oled_show_string(0, 2, ">");
		}
		// 模式5
		else if (menuflag_temp == 5)
		{
			oled_clear();
			Mode_5_Menu();
			// 返回后重新显示菜单
			oled_set_font(OLED_8X16_FONT);  
            Menu_UI(2);
            oled_show_string(0, 4, ">");
		}	

		
		/* 菜单显示更新*/
		// 判断是否需要更新
		if (key_pressed)
		{			
			switch(menuflag)
			{
				case 1:
					oled_clear();
					Menu_UI(1);
					oled_show_string(0, 4, ">");
				
					break;
				
				case 2:
					oled_clear();
					Menu_UI(1);
					oled_show_string(0, 6, ">");
				
					break;
				
				case 3:
					oled_clear();
					Menu_UI(2);
					oled_show_string(0, 0, ">");
					
					break;
				
				case 4:
					oled_clear();
					Menu_UI(2);
					oled_show_string(0, 2, ">");
				
					break;
				
				case 5:
					oled_clear();
					Menu_UI(2);
					oled_show_string(0, 4, ">");
				
					break;
			}
		}
	
	}
}

/*--------------------[E] 交互界面 [E]--------------------*/
