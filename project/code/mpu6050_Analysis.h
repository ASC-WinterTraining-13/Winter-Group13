#ifndef __MPU6050_ANALYSIS_H
#define __MPU6050_ANALYSIS_H

#include "zf_common_typedef.h"

// 解算结果
extern float Roll_Result;
extern float Yaw_Result;
extern float Pitch_Result;
extern float Angle_Result;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     零飘校准
// 使用示例     
//			// mpu6050零飘校准逻辑（此时请保持静止）
//			MPU6050_Calibration_Start();
//			while(1)  // 零飘校准循环
//			{
//				if (MPU6050_Calibration_Check() == 0)  // 零飘校准完成
//				{
//					break;  // 跳出零飘校准循环，往下执行
//				}
//				
//				// 可以考虑在这里操作OLED
//				
//				// 强制零飘校准退出
//				if(KEY_SHORT_PRESS == key_get_state(KEY_BACK)) {
//					key_clear_state(KEY_BACK);
//					break;  // 退出零飘校准模式
//				}
//				
//			}
// 备注信息     最好找个地方静止校准一下零飘
//-------------------------------------------------------------------------------------------------------------------

void MPU6050_Calibration_Start(void);
uint8_t MPU6050_Calibration_Check(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     mpu6050姿态解算
// 使用示例     
//			//调用mpu6050数据接收与解析
//			if (mpu6050_analysis_enable)
//			{
//				mpu6050_get_data();
//				mpu6050_analysis_enable = 0;
//				MPU6050_Analysis();
//			}
// 备注信息     在函数中循环调用
//-------------------------------------------------------------------------------------------------------------------	

void MPU6050_Analysis(void);

#endif
