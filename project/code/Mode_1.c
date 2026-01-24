#include "zf_device_oled.h"

#include "Key.h"
#include "Menu.h"
#include "param_config.h"

/*--------------------[S] 菜单样式 [S]--------------------*/

void Mode_1_UI(uint8_t Page)
{
	switch(Page){
		
		//第一页
		case 1:
		{
			oled_show_string(0, 0, "Mode_1");
			oled_show_string(0, 1, "===");
			oled_show_string(2, 2, " Kp:");
			oled_show_string(2, 3, " Ki:");
			oled_show_string(2, 4, " Kd:");
			oled_show_float(24, 2, angle_pid.kp, 2, 2);
			oled_show_float(24, 3, angle_pid.ki, 2, 2);
			oled_show_float(24, 4, angle_pid.kd, 2, 2);

			break;
		}
	}
}
/*--------------------[E] 菜单样式 [E]--------------------*/


//按键值初始为无效值0;返回值1、2、3、4对应（逐飞学习板）E2、E3、E4、E5;功能见宏定义
static volatile uint8_t KeyNum = 0;
#define KEY_UP			1
#define KEY_DOWN		2
#define KEY_CONFIRM		3
#define KEY_BACK		4


/*--------------------[S] 数据更改 [S]--------------------*/


void Set_Mode_1_Param(uint8_t Num)
{
	//数据更改步幅
	PID_Params_t angle_pid_step = {1.0f, 0.01f, 0.1f};
	
	 // 指向要修改的参数的指针
    float* current_param = NULL;
    float step_value = 0.0f;
    uint8_t row = 0;  // 数据对应的显示行号
    
    // 根据选项确定要修改的参数
    switch (Num)
    {
        case 1:  // Kp
            current_param = &angle_pid.kp;
            step_value = angle_pid_step.kp;
            row = 2;
            break;
            
        case 2:  // Ki
            current_param = &angle_pid.ki;
            step_value = angle_pid_step.ki;
            row = 3;
            break;
            
        case 3:  // Kd
            current_param = &angle_pid.kd;
            step_value = angle_pid_step.kd;
            row = 4;
            break;
    }
	
	oled_show_string(0, row, "=");
	
	while(1)
	{
		Key_Tick();
		KeyNum = Key_GetNum();	
		
		
		if (KeyNum == KEY_UP)
        {
            *current_param += step_value;  // 增加参数
            oled_show_float(24, row, *current_param, 2, 2);  // 更新显示
        }
        else if (KeyNum == KEY_DOWN)
        {
            *current_param -= step_value;  // 减少参数
            oled_show_float(24, row, *current_param, 2, 2);  // 更新显示
        }
        else if (KeyNum == KEY_CONFIRM || KeyNum == KEY_BACK)
        {
            // 恢复光标为 ">"
            oled_show_string(0, row, ">");
            break;  // 退出修改模式
        }
	}
}
/*--------------------[E] 数据更改 [E]--------------------*/


/*--------------------[S] 主要运行函数 [S]--------------------*/


//选项数量
#define OPT_NUM			3

//菜单选项标志位
static uint8_t mode_flag = 1;

int Mode_1_Function(void)
{
	mode_flag = 1;
	
	oled_set_font(OLED_6X8_FONT);  
	//显示菜单
	Mode_1_UI(1);
	oled_show_string(0, 2, ">");
	
	while(1)
	{
		
		
		/*--------------------[S] 交互界面 [S]--------------------*/
		
		//存储确认键被按下时mode_flag的值的临时变量，默认为无效值0
		uint8_t mode_flag_temp = 0;
		
		Key_Tick();
		KeyNum = Key_GetNum();	
				
		/*按键解析*/
		if (KeyNum == KEY_UP)
		{
			mode_flag --;
			if (mode_flag < 1)mode_flag = OPT_NUM;	
		}
		else if (KeyNum == KEY_DOWN)
		{
			mode_flag ++;
			if (mode_flag > OPT_NUM)mode_flag = 1;	
		}
		else if (KeyNum == KEY_CONFIRM)
		{
			mode_flag_temp = mode_flag;
		}
		else if (KeyNum == KEY_BACK)
		{
			oled_clear();
			oled_set_font(OLED_8X16_FONT);  
			Menu_UI(1);
			oled_show_string(0, 4, ">");
			return 0;
		}
		
		/*数据更改模式*/
		if (mode_flag_temp)
		{
			Set_Mode_1_Param(mode_flag_temp);
		}		

		
		/*菜单显示更新*/
		//判断是否需要更新
		if (KeyNum == KEY_UP || KeyNum == KEY_DOWN)
		{
			switch(mode_flag)
			{
				case 1:
					oled_clear();
					Mode_1_UI(1);
					oled_show_string(0, 2, ">");
				
					break;
				
				case 2:
					oled_clear();
					Mode_1_UI(1);
					oled_show_string(0, 3, ">");
				
					break;
				
				case 3:
					oled_clear();
					Mode_1_UI(1);
					oled_show_string(0, 4, ">");
					
					break;
				
			}
		}

		/*--------------------[E] 交互界面 [E]--------------------*/
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
	}
}

/*--------------------[E] 主要运行函数 [E]--------------------*/
