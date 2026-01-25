#include "zf_device_mpu6050.h"

#include <math.h>

#include "timer_flag.h"


/*--------------------[S] 校准函数 [S]--------------------*/

// 2秒校准需要的样本数（5ms周期）
#define CALIB_TARGET_SAMPLES    400  

//枚举定义动态校准状态
typedef enum {
    CALIB_STATE_SPARE = 0,     				// 备用/未开始状态
    CALIB_STATE_RUNNING = 1,   				// 校准进行中
    CALIB_STATE_DONE = 2       				// 校准完成
} CalibState_t;

static CalibState_t calib_state = CALIB_STATE_SPARE;  // 初始为备用状态
static uint16_t calib_count = 0;
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
//偏移值
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

//开始校准
void MPU6050_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;  // 状态设为运行中
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
	
}

//返回校准状态;0完成，1进行
uint8_t MPU6050_Calibration_Check(void)
{
	 // 如果不在校准状态，直接返回0
    if(calib_state != CALIB_STATE_RUNNING)
	{
        return 0;
    }
	
	// 检查是否有数据需要处理
	if(mpu6050_analysis_enable)
    {
        // 1. 读取原始数据
        mpu6050_get_data();
        mpu6050_analysis_enable = 0;
        
        // 2. 累加数据
        sum_gx += mpu6050_gyro_x;
        sum_gy += mpu6050_gyro_y;
        sum_gz += mpu6050_gyro_z;
        calib_count++;
        
        // 3. 检查是否达到目标样本数
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            // 计算平均值
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            
            // 状态设为完成
            calib_state = CALIB_STATE_DONE;
        }
    }
    
    // 返回当前状态
    return (calib_state == CALIB_STATE_RUNNING);
}

/*--------------------[E] 校准函数 [E]--------------------*/


/*--------------------[S] 解算函数 [S]--------------------*/

static float RollAcc = 0.0f;    		// 加速度计计算的横滚角
static float RollGyro = 0.0f;   		// 陀螺仪积分的横滚角
float Roll= 0.0f;       				// 融合后的横滚角
float Roll_Result= 0.0f;       				// 融合后的横滚角

float Yaw = 0;							//偏航角
float Yaw_Result= 0.0f;       				// 融合后的横滚角

static float PitchAcc = 0.0f;			//加速度计算的俯仰角
static float PitchGyro = 0.0f;			//陀螺仪积分的俯仰角
float Pitch = 0.0f;						//融合后的俯仰角
float Pitch_Result= 0.0f;       				// 融合后的横滚角

// 低通滤波系数（0.2 = 强滤波，0.5 = 中等，0.8 = 弱滤波）
#define FILTER_ALPHA 0.3f

void MPU6050_Analysis(void)
{
	 if(calib_state == CALIB_STATE_DONE)
	 {
		mpu6050_gyro_x -= gyro_off_x;
		mpu6050_gyro_y -= gyro_off_y;
		mpu6050_gyro_z -= gyro_off_z;
	 }
	 else 
	 {
		mpu6050_gyro_x += 12;
		mpu6050_gyro_y -= 6.5;
		mpu6050_gyro_z += 38;
	 }
	
	if(-1 < mpu6050_gyro_x && mpu6050_gyro_x < 2){mpu6050_gyro_x = 0;}
	if(-1 < mpu6050_gyro_y && mpu6050_gyro_y < 2){mpu6050_gyro_y = 0;}
	if(-1 < mpu6050_gyro_z && mpu6050_gyro_z < 2){mpu6050_gyro_z = 0;}
	
	
	// 横滚角计算
	RollAcc = atan2(mpu6050_acc_y, mpu6050_acc_z) / 3.14159 * 180;  				// 横滚角（绕X轴）
	RollGyro = Roll + mpu6050_gyro_x / 32768.0 * 2000 * 0.005;  		// 陀螺仪X轴积分
	Roll = 0.005 * RollAcc + (1 - 0.005) * RollGyro;  		// 相同互补滤波算法
	
	// 偏航角：仅陀螺仪积分（无加速度计校准，会漂移）
	Yaw += mpu6050_gyro_z / 32768.0 * 2000 * 0.005;

	// 俯仰角计算
	PitchAcc = -atan2(mpu6050_acc_x, mpu6050_acc_z) / 3.14159 * 180;  			// 俯仰角（绕Y轴）
	PitchGyro = Pitch + mpu6050_gyro_y / 32768.0 * 2000 * 0.005;  		// 陀螺仪积分（2000是量程，0.001是1ms采样间隔）
	Pitch = 0.005 * PitchAcc + (1 - 0.005) * PitchGyro;  	// 互补滤波
	
	//低通滤波
	Roll_Result = FILTER_ALPHA * Roll + (1 - FILTER_ALPHA) * Roll_Result;
	Pitch_Result = FILTER_ALPHA * Pitch + (1 - FILTER_ALPHA) * Pitch_Result;
	Yaw_Result = FILTER_ALPHA * Yaw + (1 - FILTER_ALPHA) * Yaw_Result;
}
/*--------------------[E] 解算函数 [E]--------------------*/
