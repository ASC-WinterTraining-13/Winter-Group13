#include "zf_device_oled.h"
#include "zf_device_key.h"

#include "param_config.h"
#include "param_storage.h"

/*--------------------[S] 菜单样式 [S]--------------------*/

//参数选择界面
void Core_Param_UI(uint8_t Page)
{
	switch(Page)
	{
		 //第一页
		case 1:
		{
			oled_show_string(0, 0, "Param");
			oled_show_string(0, 1, "===");
			oled_show_string(2, 2, " Angle");
			oled_show_string(2, 3, " Speed");
			oled_show_string(2, 4, " ?????");
			oled_show_string(2, 5, " ?????");
			oled_show_string(2, 6, " ?????");
			
			break;
		}
	}
}

//UI补丁：针对选择PID参数
void Core_Param_Show_PID_Num_UI(PID_t *p)
{
	oled_show_float(28, 2, p->Kp, 3, 2);
	oled_show_float(28, 3, p->Ki, 3, 2);
	oled_show_float(28, 4, p->Kd, 3, 2);
}

//PID参数更改界面
void Core_Param_Set_PID_UI(uint8_t Page)
{
    switch(Page)
	{      
        //第一页
        case 1:
        {
//            oled_show_string(0, 0, "Param");
            oled_show_string(0, 1, "===");
            oled_show_string(2, 2, " Kp:");
            oled_show_string(2, 3, " Ki:");
            oled_show_string(2, 4, " Kd:");

            break;
        }
    }
}
/*--------------------[E] 菜单样式 [E]--------------------*/

/*--------------------[S] 数据更改 [S]--------------------*/

// 同步PID结构体到Flash缓冲区（核心修复Flash存储）
static void Sync_PID_To_Buffer(PID_t *p, uint8_t PID_Num)
{
    if (p == NULL || PID_Num < 1 || PID_Num > 5) return;
    uint8_t buf_idx = (PID_Num - 1) * 3;  // 转换为缓冲区索引（1→0, 2→3, 3→6...）
    flash_union_buffer[buf_idx + 0].float_type = p->Kp;
    flash_union_buffer[buf_idx + 1].float_type = p->Ki;
    flash_union_buffer[buf_idx + 2].float_type = p->Kd;
}


void Set_Core_Param_PID(uint8_t K_Num, PID_t *p, uint8_t PID_Num)
{
    //指向要修改的参数的指针
    float* current_param = NULL;
    float step_value = 0.0f;
    uint8_t row = 0;  //数据对应的显示行号
    
    //根据选项确定要修改的参数
    switch (K_Num)
    {
        case 1:  // Kp
            current_param = &p->Kp;
            step_value = PID_STEPS[PID_Num-1][0];
            row = 2;
			oled_show_float(28, row, *current_param, 3, 2);
            break;
            
        case 2:  // Ki
            current_param = &p->Ki;
            step_value = PID_STEPS[PID_Num-1][1];
            row = 3;
			oled_show_float(28, row, *current_param, 3, 2);
            break;
            
        case 3:  // Kd
            current_param = &p->Kd;
            step_value = PID_STEPS[PID_Num-1][2];
            row = 4;
			oled_show_float(28, row, *current_param, 3, 2);
            break;
    }
    
    oled_show_string(0, row, "=");
    
    while(1)
    {              
        /*按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            *current_param += step_value;  // 增加参数
            oled_show_float(28, row, *current_param, 3, 2);  // 更新显示
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            *current_param -= step_value;  // 减少参数
            oled_show_float(28, row, *current_param, 3, 2);  // 更新显示
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
                 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
			key_clear_state(KEY_CONFIRM);
			key_clear_state(KEY_BACK);
            // 恢复光标为 ">"            
            oled_show_string(0, row, ">");
			Sync_PID_To_Buffer(p, PID_Num);
            Param_Save();
            break;  // 退出修改模式
        }
    }
}
/*--------------------[E] 数据更改 [E]--------------------*/


/*--------------------[S] 交互界面 [S]--------------------*/

/*[模式内菜单子界面]*/

//参数设置选项数量
#define OPT_NUM         3

int Set_Core_Param(uint8_t PID_Num)
{
    //参数设置选项光标 标志位
    static uint8_t Param_flag = 1;
    
    //显示
    oled_set_font(OLED_6X8_FONT);  
    Core_Param_Set_PID_UI(1);
    oled_show_string(0, 2, ">");
    
    while(1)
    {
        //存储确认键被按下时Param_flag的值的临时变量，默认为无效值0
        uint8_t Param_flag_temp = 0;
        
        //上/下按键是否被按下过
        uint8_t key_pressed = 0;  
                
        /*按键解析*/
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
			oled_clear();
            //返回上一级界面
            return 0;
        }
        
        /*数据更改模式*/
        if (Param_flag_temp)
        {
            switch(PID_Num)
			{
				case 1:
					Set_Core_Param_PID(Param_flag_temp, &Angle_PID,PID_Num);
					break;
				
				case 2:
					Set_Core_Param_PID(Param_flag_temp, &Speed_PID,PID_Num);
					break;
								
				case 3:
					Set_Core_Param_PID(Param_flag_temp, &TEMP_888_FUNC_3_PID,PID_Num);
					break;			
				
				case 4:
					Set_Core_Param_PID(Param_flag_temp, &TEMP_888_FUNC_4_PID,PID_Num);
					break;			
				
				case 5:
					Set_Core_Param_PID(Param_flag_temp, &TEMP_888_FUNC_5_PID,PID_Num);
					break;
				
			}
        }        
        
        /*显示更新*/
        //判断界面是否需要更新
        if (key_pressed)
        {
			oled_clear();
            Core_Param_Set_PID_UI(1);
            oled_show_string(0, Param_flag+1, ">");
        }    
    }
}

//参数设置选项数量
#define CORE_PARAM_NUM         5

int Core_Param_Menu(void)
{
	//参数数组选项光标 标志位
    static uint8_t Core_Param_flag = 1;
	
	oled_set_font(OLED_6X8_FONT); 
	Core_Param_UI(1);
	oled_show_string(0, 2, ">");
	
	while(1)
	{
		//存储确认键被按下时Core_Param_flag的值的临时变量，默认为无效值0
        uint8_t Core_Param_flag_temp = 0;
        
        //上/下按键是否被按下过
        uint8_t key_pressed = 0;
        
        /*按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_pressed = 1;
            key_clear_state(KEY_UP);
            Core_Param_flag --;
            if (Core_Param_flag < 1)Core_Param_flag = CORE_PARAM_NUM;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_pressed = 1;
            key_clear_state(KEY_DOWN);
            Core_Param_flag ++;
            if (Core_Param_flag > CORE_PARAM_NUM)Core_Param_flag = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Core_Param_flag_temp = Core_Param_flag;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            oled_clear();
            
            return 0;
        }
        
        /*页面跳转*/
        if (1 <= Core_Param_flag_temp && Core_Param_flag_temp <= 5)
        {
            oled_clear();
			Core_Param_Set_PID_UI(1);
			oled_show_string(0, 2, ">");
			switch (Core_Param_flag_temp)
			{
				case 1:
					oled_show_string(0, 0, "Angle_PID");
					Core_Param_Show_PID_Num_UI(&Angle_PID);					
					break;
				
				case 2:
					oled_show_string(0, 0, "Speed_PID");
					Core_Param_Show_PID_Num_UI(&Speed_PID);
					break;
				
				case 3:
					oled_show_string(0, 0, "?????_PID");
					Core_Param_Show_PID_Num_UI(&TEMP_888_FUNC_3_PID);
					break;
				
				case 4:
					oled_show_string(0, 0, "?????_PID");
					Core_Param_Show_PID_Num_UI(&TEMP_888_FUNC_4_PID);
					break;
				
				case 5:
					oled_show_string(0, 0, "?????_PID");
					Core_Param_Show_PID_Num_UI(&TEMP_888_FUNC_5_PID);
					break;
				
			}
			Set_Core_Param(Core_Param_flag_temp);
			
			oled_clear();
            Core_Param_UI(1);
            oled_show_string(0, Core_Param_flag+1, ">");
        }

        /*显示更新*/
        if (key_pressed)
        {
			if (1 <= Core_Param_flag && Core_Param_flag <= 5)
			{
				oled_clear();
				Core_Param_UI(1);
				oled_show_string(0, Core_Param_flag+1, ">");
			}
        }    
		
		
	}
	
}
/*--------------------[S] 交互界面 [S]--------------------*/
