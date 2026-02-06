#include "zf_device_key.h"

#include "param_config.h"
#include "param_storage.h"
#include "OLED.h"

/*******************************************************************************************************************/
/*[S] 菜单样式 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]参数选择界面
void Core_Param_UI(uint8_t Page)
{
	switch(Page)
	{
		 // 第一页
		case 1:
		{
			OLED_ShowString(0  , 0  , "Param", OLED_6X8);
			OLED_ShowString(0  , 8  , "=====================", OLED_6X8);
			OLED_ShowString(10 , 16 , "Rate__PID", OLED_6X8);
			OLED_ShowString(10 , 24 , "Angle_PID", OLED_6X8);
			OLED_ShowString(10 , 32 , "Speed_PID", OLED_6X8);
			OLED_ShowString(10 , 40 , "Turn__PID", OLED_6X8);
			OLED_ShowString(10 , 48 , "Track_PID", OLED_6X8);
			
			break;
		}
	}
}

// [三级界面]PID参数更改界面
void Core_Param_Set_PID_UI(uint8_t Page, PID_t *p)
{
    switch(Page)
	{      
        // 第一页
        case 1:
        {
			OLED_ShowString(0  , 8  , "=====================", OLED_6X8);
			OLED_Printf(10, 16, OLED_6X8, "Kp:%4.2f    ", p->Kp);
			OLED_Printf(10, 24, OLED_6X8, "Ki:%4.2f    ", p->Ki);
			OLED_Printf(10, 32, OLED_6X8, "Kd:%4.2f    ", p->Kd);

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
    // 指向要修改的参数的指针
    float* current_param = NULL;
    float step_value = 0.0f;
    uint8_t row = 0;  // 数据对应的显示行号
    
    // 根据选项确定要修改的参数
    switch (K_Num)
    {
        case 1:  // Kp
            current_param = &p->Kp;
            step_value = PID_STEPS[PID_Num-1][0];
            row = 16;
		
            break;           
        case 2:  // Ki
            current_param = &p->Ki;
            step_value = PID_STEPS[PID_Num-1][1];
            row = 24;
		
            break;           
        case 3:  // Kd
            current_param = &p->Kd;
            step_value = PID_STEPS[PID_Num-1][2];
            row = 32;

            break;
    }
    
	OLED_ShowString(0 , row , "=", OLED_6X8);
	OLED_Update();
    
    while(1)
    {              
        /* 按键解析*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            *current_param += step_value;  // 增加参数
			OLED_Printf(28, row, OLED_6X8, "%4.2f   ", *current_param);
			OLED_Update();
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            *current_param -= step_value;  // 减少参数
            OLED_Printf(28, row, OLED_6X8, "%4.2f   ", *current_param);
			OLED_Update();
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
                 KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
			key_clear_state(KEY_CONFIRM);
			key_clear_state(KEY_BACK);
            
			Sync_PID_To_Buffer(p, PID_Num);
            Param_Save();
			// 恢复光标为 ">"            
            OLED_ShowString(0 , row , ">", OLED_6X8);
			OLED_Update();
			
            break;  // 退出修改模式
        }
    }
}
/*******************************************************************************************************************/
/*[E] 参数更改 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/********************************************************************************************************************/
/*[S] 交互界面 [S]--------------------------------------------------------------------------------------------------*/
/********************************************************************************************************************/

// [三级界面]PID参数更改界面

// 参数设置选项数量
#define OPT_NUM         3

int Set_Core_Param(uint8_t PID_Num)
{
    // 参数设置选项光标 标志位
    uint8_t Param_flag = 1;
    
    // 显示
    OLED_ShowString(0 , 16 , ">", OLED_6X8);
    OLED_Update();
	
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
            //返回上一级界面
            return 0;
        }
		
        
        /* 数据更改模式*/
        if (Param_flag_temp)
        {
            switch(PID_Num)
			{
				case 1:
					Set_Core_Param_PID(Param_flag_temp, &Rate__PID, PID_Num);
					break;
				
				case 2:
					Set_Core_Param_PID(Param_flag_temp, &Angle_PID, PID_Num);
					break;
								
				case 3:
					Set_Core_Param_PID(Param_flag_temp, &Speed_PID, PID_Num);
					break;			
				
				case 4:
					Set_Core_Param_PID(Param_flag_temp, &Turn__PID, PID_Num);
					break;			
				
				case 5:
					Set_Core_Param_PID(Param_flag_temp, &Track_PID, PID_Num);
					break;
				
			}
        }        
        
		
        /* 显示更新*/
        if (key_pressed)
        {
			// 更新光标位置
			OLED_ShowString(0 , 16 , " ", OLED_6X8);
			OLED_ShowString(0 , 24 , " ", OLED_6X8);
			OLED_ShowString(0 , 32 , " ", OLED_6X8);
			OLED_ShowString(0 , (Param_flag+1)*8 , ">", OLED_6X8);
			OLED_Update();
        }    
    }
}

// [二级界面]参数选择界面

// 参数设置选项数量
#define CORE_PARAM_NUM         5

int Core_Param_Menu(void)
{
	// 参数数组选项光标 标志位
    uint8_t Core_Param_flag = 1;
	
	Core_Param_UI(1);
	OLED_ShowString(0 , 16 , ">", OLED_6X8);
	OLED_Update();
	
	while(1)
	{
		// 存储确认键被按下时Core_Param_flag的值的临时变量，默认为无效值0
        uint8_t Core_Param_flag_temp = 0;
        
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;
        
		
        /* 按键解析*/
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
            
            return 0;
        }
		
        
        /* 页面跳转*/
        if (1 <= Core_Param_flag_temp && Core_Param_flag_temp <= 5)
        {
            OLED_Clear();
			switch (Core_Param_flag_temp)
			{
				case 1:
					OLED_ShowString(0 , 0 , "Rate__PID", OLED_6X8);
					Core_Param_Set_PID_UI(1, &Rate__PID);
					OLED_ShowString(0 , 16 , ">", OLED_6X8);
					OLED_Update();
					break;				
				case 2:
					OLED_ShowString(0 , 0 , "Angle_PID", OLED_6X8);
					Core_Param_Set_PID_UI(1, &Angle_PID);
					OLED_ShowString(0 , 16 , ">", OLED_6X8);
					OLED_Update();
					break;
				
				case 3:
					OLED_ShowString(0 , 0 , "Speed_PID", OLED_6X8);
					Core_Param_Set_PID_UI(1, &Speed_PID);
					OLED_ShowString(0 , 16 , ">", OLED_6X8);
					OLED_Update();
					break;
				
				case 4:
					OLED_ShowString(0 , 0 , "Turn__PID", OLED_6X8);
					Core_Param_Set_PID_UI(1, &Turn__PID);
					OLED_ShowString(0 , 16 , ">", OLED_6X8);
					OLED_Update();
					break;
				
				case 5:
					OLED_ShowString(0 , 0 , "Track_PID", OLED_6X8);
					Core_Param_Set_PID_UI(1, &Track_PID);
					OLED_ShowString(0 , 16 , ">", OLED_6X8);
					OLED_Update();
					break;
				
			}
			Set_Core_Param(Core_Param_flag_temp);
			
			// 重新显示菜单
			OLED_Clear();
            Core_Param_UI(1);
            OLED_ShowString(0 , (Core_Param_flag_temp+1)*8 , ">", OLED_6X8);
			OLED_Update();
        }

		
        /* 显示更新*/
        if (key_pressed)
        {
			if (1 <= Core_Param_flag && Core_Param_flag <= 5)
			{
				OLED_Clear();
				Core_Param_UI(1);
				OLED_ShowString(0 , (Core_Param_flag+1)*8 , ">", OLED_6X8);
				OLED_Update();
			}
        } 
	}	
}
/*******************************************************************************************************************/
/*[E] 交互界面 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
