#include "zf_device_mpu6050.h"

#include <math.h>

static float RollAcc = 0.0f;    		// 加速度计计算的横滚角
static float RollGyro = 0.0f;   		// 陀螺仪积分的横滚角
float Roll = 0.0f;       				// 融合后的横滚角

float Yaw = 0;							//偏航角

static float PitchAcc = 0.0f;			//加速度计算的俯仰角
static float PitchGyro = 0.0f;			//陀螺仪积分的俯仰角
float Pitch = 0.0f;						//融合后的俯仰角

void MPU6050_Analysis(void)
{
	mpu6050_gyro_x += 12;
	mpu6050_gyro_y -= 7;
	mpu6050_gyro_z += 39;
	
	if(-1 < mpu6050_gyro_x && mpu6050_gyro_x < 2){mpu6050_gyro_x = 0;}
	if(-1 < mpu6050_gyro_y && mpu6050_gyro_y < 2){mpu6050_gyro_y = 0;}
	if(-1 < mpu6050_gyro_z && mpu6050_gyro_z < 2){mpu6050_gyro_z = 0;}
	
	
	// 横滚角计算
	RollAcc = atan2(mpu6050_acc_y, mpu6050_acc_z) / 3.14159 * 180;  				// 横滚角（绕X轴）
	RollGyro = Roll + mpu6050_gyro_x / 32768.0 * 2000 * 0.005;  		// 陀螺仪X轴积分
	Roll = 0.005 * RollAcc + (1 - 0.005) * RollGyro;  		// 相同互补滤波算法
	
	// 偏航角：仅陀螺仪积分（无加速度计校准，会漂移）
	if (mpu6050_gyro_z <= -2 || 2 <= mpu6050_gyro_z){Yaw += mpu6050_gyro_z / 32768.0 * 2000 * 0.005;}

	// 俯仰角计算
	PitchAcc = -atan2(mpu6050_acc_x, mpu6050_acc_z) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
	PitchGyro = Pitch + mpu6050_gyro_y / 32768.0 * 2000 * 0.005;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
	Pitch = 0.005 * PitchAcc + (1 - 0.005) * PitchGyro;  	// 互补滤波
	
}
