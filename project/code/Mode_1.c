#include "zf_device_oled.h"
#include "zf_device_key.h"

#include "Menu.h"
#include "param_config.h"
#include "param_storage.h"

#include "zf_device_mpu6050.h"
#include "timer_flag.h"
#include "mpu6050_Analysis.h"

/*--------------------[S] 菜单样式 [S]--------------------*/

//模式内界面
void Mode_1_Menu_UI(void)
{
    oled_show_string(0, 0, "Mode_1");
    oled_show_string(0, 2, "===");
    oled_show_string(2, 4, " Start");
    oled_show_string(2, 6, " Param");
}

//模式内参数设置界面
void Mode_1_Set_Param_UI(uint8_t Page)
{
    switch(Page){
        
        //第一页
        case 1:
        {
            oled_show_string(0, 0, "Param");
            oled_show_string(0, 1, "===");
            oled_show_string(2, 2, " Kp:");
            oled_show_string(2, 3, " Ki:");
            oled_show_string(2, 4, " Kd:");
            oled_show_float(28, 2, ANGLE_KP, 2, 2);
            oled_show_float(28, 3, ANGLE_KI, 2, 2);
            oled_show_float(28, 4, ANGLE_KD, 2, 2);

            break;
        }
    }
}
/*--------------------[E] 菜单样式 [E]--------------------*/


/*--------------------[S] 数据更改 [S]--------------------*/

void Set_Mode_1_Param(uint8_t Num)
{
    //数据更改步幅
    PID_Params_t angle_pid_step = {1.0f, 0.01f, 0.1f};
    
    //指向要修改的参数的指针
    float* current_param = NULL;
    float step_value = 0.0f;
    uint8_t row = 0;  //数据对应的显示行号
    
    //根据选项确定要修改的参数
    switch (Num)
    {
        case 1:  // Kp
            current_param = &ANGLE_KP;
            step_value = angle_pid_step.kp;
            row = 2;
            break;
            
        case 2:  // Ki
            current_param = &ANGLE_KI;
            step_value = angle_pid_step.ki;
            row = 3;
            break;
            
        case 3:  // Kd
            current_param = &ANGLE_KD;
            step_value = angle_pid_step.kd;
            row = 4;
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
            oled_show_float(28, row, *current_param, 2, 2);  // 更新显示
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            *current_param -= step_value;  // 减少参数
            oled_show_float(28, row, *current_param, 2, 2);  // 更新显示
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
/*--------------------[E] 数据更改 [E]--------------------*/


/*--------------------[S] 交互界面 [S]--------------------*/

/*[模式内菜单子界面]*/

//参数设置选项数量
#define OPT_NUM         3

int Mode_1_Set_Param(void)
{
    //参数设置选项光标 标志位
    static uint8_t Param_flag = 1;
    
    //显示
    oled_set_font(OLED_6X8_FONT);  
    Mode_1_Set_Param_UI(1);
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
            //返回上一级界面
            return 0;
        }
        
        /*数据更改模式*/
        if (Param_flag_temp)
        {
            Set_Mode_1_Param(Param_flag_temp);
        }        
        
        /*显示更新*/
        //判断界面是否需要更新
        if (key_pressed)
        {
            switch(Param_flag)
            {
                case 1:
                    oled_clear();
                    Mode_1_Set_Param_UI(1);
                    oled_show_string(0, 2, ">");
                
                    break;
                
                case 2:
                    oled_clear();
                    Mode_1_Set_Param_UI(1);
                    oled_show_string(0, 3, ">");
                
                    break;
                
                case 3:
                    oled_clear();
                    Mode_1_Set_Param_UI(1);
                    oled_show_string(0, 4, ">");
                    
                    break;                
            }
        }    
    }
}

//方便模式内菜单母界面调用
int Mode_1_Running(void);

/*[模式内菜单母界面]*/

int Mode_1_Menu(void)
{
    //模式菜单选项光标 标志位
    static uint8_t Mode_Menu_flag = 1;
    
    //显示
    Mode_1_Menu_UI();
    oled_show_string(0, 4, ">");
    
    while(1)
    {
        //存储确认键被按下时Mode_Menu_flag的值的临时变量，默认为无效值0
        uint8_t Mode_Menu_flag_temp = 0;
        
        //上/下按键是否被按下过
        uint8_t key_pressed = 0;     
        
        /*按键解析*/
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
            oled_clear();
            oled_set_font(OLED_8X16_FONT);  
            Menu_UI(1);
            oled_show_string(0, 4, ">");
            
            return 0;
        }
        
        /*页面跳转*/
        if (Mode_Menu_flag_temp == 1)
        {
            oled_clear();
            Mode_1_Running();
            //返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_1_Menu_UI();
            oled_show_string(0, 4, ">");
        }
        if (Mode_Menu_flag_temp == 2)
        {
            oled_clear();
            Mode_1_Set_Param();
            //返回后重新显示菜单
            oled_clear();
            oled_set_font(OLED_8X16_FONT); 
            Mode_1_Menu_UI();
            oled_show_string(0, 6, ">");
        }
        
        /*显示更新*/
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
/*--------------------[E] 交互界面 [E]--------------------*/


/*--------------------[S] 小车运行界面 [S]--------------------*/

/*[模式内菜单子界面]*/

int Mode_1_Running(void)
{
	oled_set_font(OLED_6X8_FONT);

	oled_show_string(0, 1, "R :");
	oled_show_string(0, 2, "Y :");
	oled_show_string(0, 3, "P :");
	oled_show_string(0, 4, "GX:");
	oled_show_string(0, 5, "GY:");
	oled_show_string(0, 6, "GZ:");
    
	//校准逻辑
	MPU6050_Calibration_Start();
	while(1)  // 校准循环
    {
        if (MPU6050_Calibration_Check() == 0)  // 校准完成
        {
            break;  //跳出校准循环，往下执行
        }
        
        //可以考虑在这里操作OLED
        
        //强制校准退出
        if(KEY_SHORT_PRESS == key_get_state(KEY_BACK)) {
            key_clear_state(KEY_BACK);
            break;  // 退出整个模式
        }
        
        //防止刷新过快
        system_delay_ms(1);
    }
	
	oled_show_string(0, 0, "Running");
	
    while(1)
    {  
		
        
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            // 处理上键
        }

        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            // 处理下键
        }

        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            // 处理确认键
        }

        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 处理返回键
            return 0;
        }
		
		if (mpu6050_analysis_enable)
		{
			mpu6050_get_data();
			mpu6050_analysis_enable = 0;
			MPU6050_Analysis();
		}

		
		oled_show_float(18, 1, Roll_Result, 3, 3);
		oled_show_float(18, 2, Yaw_Result, 3, 3);
		oled_show_float(18, 3, Pitch_Result, 3, 3);
		oled_show_int(18, 4, mpu6050_gyro_x, 4);
		oled_show_int(18, 5, mpu6050_gyro_y, 4);
		oled_show_int(18, 6, mpu6050_gyro_z, 4);
		
    }
}
/*--------------------[E] 小车运行界面 [E]--------------------*/
