#include "zf_device_key.h"
#include "zf_device_mpu6050.h"

#include "param_config.h"
#include "param_storage.h"
#include "mpu6050_Analysis.h"
#include "OLED.h"

int SandBox_Page(void)
{
	OLED_ShowString(0 , 0 , "CAli"  , OLED_6X8);
	OLED_Update();
	
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
	
	OLED_ShowString(0 , 0 , "Done"  , OLED_6X8);
	OLED_Update();
	
	OLED_ShowString(0 , 8, "Roll :" , OLED_6X8);
	OLED_ShowString(0 , 16, "Yaw  :", OLED_6X8);
	OLED_ShowString(0 , 24, "Pitch:", OLED_6X8);
	OLED_ShowString(0 , 32, "GX:"   , OLED_6X8);
	OLED_ShowString(0 , 40, "GY:"   , OLED_6X8);
	OLED_ShowString(0 , 48, "GZ:"   , OLED_6X8);
	OLED_ShowString(64, 32, "AX:"   , OLED_6X8);
	OLED_ShowString(64, 40, "AY:"   , OLED_6X8);
	OLED_ShowString(64, 48, "AZ:"   , OLED_6X8);
	OLED_ShowString(0, 56, "Rate:"  , OLED_6X8);
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
            
            return 0;
        }	
		
		//调用mpu6050数据接收与解析
		if (mpu6050_analysis_enable)
		{
			mpu6050_get_data();
			mpu6050_analysis_enable = 0;
			MPU6050_Analysis();
		}
		
		OLED_Printf(36, 8 , OLED_6X8, "%4.3f", Roll_Result);
		OLED_Printf(36, 16, OLED_6X8, "%4.3f", Yaw_Result);
		OLED_Printf(36, 24, OLED_6X8, "%4.3f", Pitch_Result);	
//		OLED_Printf(18, 32, OLED_6X8, "%d", mpu6050_gyro_x);
//		OLED_Printf(18, 40, OLED_6X8, "%d", mpu6050_gyro_y);
//		OLED_Printf(18, 48, OLED_6X8, "%d", mpu6050_gyro_z);
//		OLED_Printf(82, 32, OLED_6X8, "%d", mpu6050_acc_x);
//		OLED_Printf(82, 40, OLED_6X8, "%d", mpu6050_acc_y);
//		OLED_Printf(82, 48, OLED_6X8, "%d", mpu6050_acc_z);
		OLED_Printf(30, 56, OLED_6X8, "%4.3f", GyroRate_Result);
		OLED_Update();
		
		
	}

}