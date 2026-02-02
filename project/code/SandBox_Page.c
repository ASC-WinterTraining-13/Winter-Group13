#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "param_config.h"
#include "param_storage.h"
#include "mpu6050_Analysis.h"

#include "BuzzerAndLED.h"

int SandBox_Page(void)
{
	oled_set_font(OLED_8X16_FONT);
	oled_show_string(0, 0, "Cali");
	
	// mpu6050零飘校准逻辑（此时请保持静止）
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
    }
	
	oled_show_string(0, 0, "Done");
	
	oled_show_string(0, 2, "Roll :");
	oled_show_string(0, 4, "Yaw  :");
	oled_show_string(0, 6, "Pitch:");
//	oled_show_string(0, 4, "GX:");
//	oled_show_string(0, 5, "GY:");
//	oled_show_string(0, 6, "GZ:");
	
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
            
            return 0;
        }	
		
		//调用mpu6050数据接收与解析
		if (mpu6050_analysis_enable)
		{
			mpu6050_get_data();
			mpu6050_analysis_enable = 0;
			MPU6050_Analysis();
		}
		
		oled_show_float(64, 2, Roll_Result , 3, 3);
		oled_show_float(64, 4, Yaw_Result  , 3, 3);
		oled_show_float(64, 6, Pitch_Result, 3, 3);
		
//		oled_show_int(18, 4, mpu6050_gyro_x, 4);
//		oled_show_int(18, 5, mpu6050_gyro_y, 4);
//		oled_show_int(18, 6, mpu6050_gyro_z, 4);
//		oled_show_int(82, 4, mpu6050_acc_x, 4);
//		oled_show_int(82, 5, mpu6050_acc_y, 4);
//		oled_show_int(82, 6, mpu6050_acc_z, 4);
		
	}

}