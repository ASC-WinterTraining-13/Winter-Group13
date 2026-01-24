#include "zf_device_oled.h"

#include "Key.h"
/*--------------------[S] 外设初始化 [S]--------------------*/

void Peripheral_Init(void)
{
	//（七针脚）OLED初始化
    oled_init();
    //可以使用的字体为6X8和8X16,点阵坐标等效是基于6X8标定的
    oled_set_font(OLED_8X16_FONT);    
	oled_clear();
	
	Key_Init();
}
/*--------------------[E] 外设初始化 [E]--------------------*/


/*--------------------[S] 菜单样式 [S]--------------------*/

void Menu_UI(uint8_t Page)
{
	switch(Page){
		
		//第一页
		case 1:
		{
			oled_show_string(0, 0, "MENU");
			oled_show_string(0, 2, "===");
			oled_show_string(0, 4, " Mode_1");
			oled_show_string(0, 6, " Mode_2");

			break;
		}
		
		//第二页
		case 2:
		{
			oled_show_string(0, 0, " Mode_3");
			oled_show_string(0, 2, " Mode_4");
			oled_show_string(0, 4, " Mode_5");
			
			break;
		}
	}
	
}
/*--------------------[E] 菜单样式 [E]--------------------*/


/*--------------------[S] 交互界面 [S]--------------------*/

//按键值初始为无效值0;返回值1、2、3、4对应（逐飞学习板）E2、E3、E4、E5;功能见宏定义
volatile uint8_t KeyNum = 0;
#define KEY_UP			1
#define KEY_DOWN		2
#define KEY_CONFIRM		3
#define KEY_BACK		4

//菜单选项标志位
uint8_t menuflag = 1;

void Menu_Show(void)
{
	Menu_UI(1);
	oled_show_string(0, 4, ">");
	
	while(1)
	{
		//存储确认键被按下时setflag的值的临时变量，默认为无效值0
		uint8_t menuflag_temp = 0;
		
		Key_Tick();
		KeyNum = Key_GetNum();	
			
		
		if (KeyNum == KEY_UP)
		{
			menuflag --;
			if (menuflag < 1)menuflag = 5;
	
		}
		else if (KeyNum == KEY_DOWN)
		{
			menuflag ++;
			if (menuflag > 5)menuflag = 1;	
		}
		else if (KeyNum == KEY_CONFIRM)
		{
			menuflag_temp = menuflag;
		}
		else if (KeyNum == KEY_BACK)
		{
			//
		}
		
		if (menuflag_temp == 1){}
		else if (menuflag_temp == 2){}
		else if (menuflag_temp == 3){}
		else if (menuflag_temp == 4){}
		else if (menuflag_temp == 5){}	

			
		if (KeyNum == KEY_UP || KeyNum == KEY_DOWN)
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
