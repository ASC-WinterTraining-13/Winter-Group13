#ifndef __MPU6050_ANALYSIS_H
#define __MPU6050_ANALYSIS_H

#include "zf_common_typedef.h"

extern float Roll_Result;
extern float Yaw_Result;
extern float Pitch_Result;
extern float Angle_Result;
extern float gyro_pitch_rate;

void MPU6050_Calibration_Start(void);
uint8_t MPU6050_Calibration_Check(void);
void MPU6050_Analysis(void);

#endif
