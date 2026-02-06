#include "zf_device_mpu6050.h"

#include <math.h>

#include "param_config.h"

//Angle选择;1-Pitch，0-Roll
#define USE_PITCH_AS_ANGLE	1


/*******************************************************************************************************************/
/*[S] 零飘校准 [S]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 2秒校准需要的样本数（5ms周期）
#define CALIB_TARGET_SAMPLES    400  

// 枚举定义动态校准状态
typedef enum {
    CALIB_STATE_SPARE   = 0,     				// 备用/未开始状态
    CALIB_STATE_RUNNING = 1,   					// 校准进行中
    CALIB_STATE_DONE    = 2       				// 校准完成
} CalibState_t;

static CalibState_t calib_state = CALIB_STATE_SPARE;  // 初始为备用状态
static uint16_t calib_count = 0;
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
// 偏移值
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

// 开始校准
void MPU6050_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;  // 状态设为运行中
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
	
}

// 返回校准状态;0完成，1进行
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
        // 读取原始数据
        mpu6050_get_data();
        mpu6050_analysis_enable = 0;
        
        //累加原始数据
        sum_gx += mpu6050_gyro_x;
        sum_gy += mpu6050_gyro_y;
        sum_gz += mpu6050_gyro_z;
        calib_count++;
        
        // 检查是否达到目标样本数
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            // 计算平均值（修正零飘系数）
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;       
			
            // 校准状态设为完成
            calib_state = CALIB_STATE_DONE;
			
        }
    }
    
    // 返回当前校准状态
    return (calib_state == CALIB_STATE_RUNNING);
}

/*******************************************************************************************************************/
/*[E] 零飘校准 [E]-------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 卡尔曼滤波 [S]-----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 卡尔曼滤波开关：1=使用卡尔曼滤波，0=使用互补滤波
#define USE_KALMAN_FILTER 1

//Q_angle（角度过程噪声）
//数值越大，越信任陀螺仪（动态响应快，但噪声大）；
//数值越小，越信任加速度计（静态准，但动态延迟大）；
//平衡车推荐初始值：0.001f~0.005f

//Q_bias（陀螺仪零飘过程噪声）
//数值越大，对零飘的动态补偿越灵敏，但容易引入抖动；
//数值越小，零飘补偿越平滑，但响应慢；
//平衡车推荐初始值：0.003f~0.01f

//R_measure（加速度计测量噪声）
//数值越大，越不信任加速度计（抗振动干扰能力强，但静态误差大）；
//数值越小，越信任加速度计（静态准，但易受动态加速度干扰）；
//平衡车推荐初始值：0.03f~0.1f

// 卡尔曼滤波结构体定义
typedef struct {
    float Q_angle;    		// 角度过程噪声协方差
    float Q_bias;     		// 偏差过程噪声协方差
    float R_measure;  		// 测量噪声协方差
    
    float angle;      		// 滤波后角度
    float bias;       		// 陀螺仪偏差
    float rate;       		// 去偏后的角速度
    
    float P[2][2];    		// 误差协方差矩阵
} KalmanFilter;

// 定义Roll/Pitch轴卡尔曼滤波器实例
static KalmanFilter kf_roll = {0};
static KalmanFilter kf_pitch = {0};

// 卡尔曼滤波初始化函数
static void Kalman_Init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure)
{
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

// 卡尔曼滤波计算核心函数
static float Kalman_Calculate(KalmanFilter* kf, float newAngle, float newRate, float dt)
{
    // 预测步骤
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    
    // 更新误差协方差矩阵
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // 更新步骤
    float S = kf->P[0][0] + kf->R_measure;  	// 创新协方差
    float K[2];                             	// 卡尔曼增益
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    float y = newAngle - kf->angle;         	// 角度残差
    kf->angle += K[0] * y;                  	// 更新角度
    kf->bias += K[1] * y;                   	// 更新偏差
    
    // 更新后验误差协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// 动态调整R_measure（根据加速度计模长判断运动状态）
static float Get_Dynamic_Rmeasure(float ax, float ay, float az)
{
    // 计算加速度模长（理想值为1g，约16384 LSB/g）
    float acc_mag = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
    // 归一化到1g参考系
    acc_mag = acc_mag / 16384.0f;
    
    // 静态（模长接近1g）：信任加速度计，R小
    if(fabs(acc_mag - 1.0f) < 0.1f)
    {
        return 0.03f;  // 静态R值
    }
    // 动态（模长偏离1g）：不信任加速度计，R大
    else
    {
        return 0.3f;   // 动态R值
    }
}

/*******************************************************************************************************************/
/*[E] 卡尔曼滤波 [E]-----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 姿态解算 + 滤波 [S]------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

static float RollAcc = 0.0f;    				// 加速度计计算的横滚角
static float RollGyro = 0.0f;   				// 陀螺仪积分的横滚角
static float Roll = 0.0f;       				// 融合后的横滚角

static float Yaw = 0;							//偏航角

static float PitchAcc = 0.0f;					//加速度计算的俯仰角
static float PitchGyro = 0.0f;					//陀螺仪积分的俯仰角
static float Pitch = 0.0f;						//融合后的俯仰角


static float Roll_Temp = 0.0f;       			// 横滚角 中间处理值（需要保留历史记录）
static float Yaw_Temp = 0.0f;       			// 偏航角 中间处理值（需要保留历史记录）
static float Pitch_Temp = 0.0f;       			// 俯仰角 中间处理值（需要保留历史记录）

float Roll_Result     = 0.0f;       			// 横滚角 最终调用值
float Yaw_Result      = 0.0f;       			// 偏航角 最终调用值
float Pitch_Result    = 0.0f;       			// 俯仰角 最终调用值

float Angle_Result    = 0.0f;					// 倾斜角 最终调用值
float GyroRate_Result = 0.0f;					// 角速度 最终调用值

// 低通滤波系数（0.2 = 强滤波，0.5 = 中等，0.8 = 弱滤波）
#define MPU6050_LOW_PASS_FILTER 0.3f
// 输出死区系数
#define MPU6050_OUTPUT_DEAD_ZONE 0.05f
// 采样时间（5ms = 0.005s）
const float MPU6050_SAMPLE_DT = 0.005f;

// 固化解算系数
// 弧度转角度
const float mpu6050_const_data1 = (1.0f / 3.14159f) * 180.0f;
// 陀螺仪积分系数
const float mpu6050_const_data2 = (1.0f / 32768.0f) * 2000.0f;

void MPU6050_Analysis(void)
{
	// 静态初始化卡尔曼滤波器（仅第一次执行）
    static uint8_t kalman_init_flag = 0;
    if(kalman_init_flag == 0 && USE_KALMAN_FILTER)
    {
        // 卡尔曼参数初始化（平衡车推荐值）
        Kalman_Init(&kf_roll, 0.001f, 0.003f, 0.03f);
        Kalman_Init(&kf_pitch, 0.001f, 0.003f, 0.03f);
        kalman_init_flag = 1;
    }
	
	
	// 尝试使用校准的系数修正原始数据
	if(calib_state == CALIB_STATE_DONE)
	{
		mpu6050_gyro_x -= gyro_off_x;
		mpu6050_gyro_y -= gyro_off_y;
		mpu6050_gyro_z -= gyro_off_z;
	}
	else 
	// 使用固定值修正原始数据
	{
		mpu6050_gyro_x += 65.2574;
		mpu6050_gyro_y += 1.2424;
		mpu6050_gyro_z -= 4.2224;
	}
	
	GyroRate_Result = mpu6050_gyro_x;
	
	// 输入死区
	if(-2 < mpu6050_gyro_x && mpu6050_gyro_x < 2){mpu6050_gyro_x = 0;}
	if(-2 < mpu6050_gyro_y && mpu6050_gyro_y < 2){mpu6050_gyro_y = 0;}
	if(-2 < mpu6050_gyro_z && mpu6050_gyro_z < 2){mpu6050_gyro_z = 0;}
	
	
    // 计算陀螺仪角速度（转换为 °/s）
    float gyro_roll_rate  = (float)mpu6050_gyro_x * mpu6050_const_data2;
    float gyro_pitch_rate = (float)mpu6050_gyro_y * mpu6050_const_data2;
	
	// 横滚角加速度计计算
	RollAcc   = atan2f((float)mpu6050_acc_y, (float)mpu6050_acc_z) * mpu6050_const_data1;
	// 俯仰角加速度计计算
	PitchAcc  = -atan2f((float)mpu6050_acc_x, (float)mpu6050_acc_z) * mpu6050_const_data1;
	
#if USE_KALMAN_FILTER
    // 卡尔曼滤波模式（Roll/Pitch轴）
    // 动态调整R_measure（根据加速度计状态）
    kf_roll.R_measure  = Get_Dynamic_Rmeasure(mpu6050_acc_x, mpu6050_acc_y, mpu6050_acc_z);
    kf_pitch.R_measure = kf_roll.R_measure;
    
    // 卡尔曼滤波计算
    Roll  = Kalman_Calculate(&kf_roll, RollAcc, gyro_roll_rate, MPU6050_SAMPLE_DT);
    Pitch = Kalman_Calculate(&kf_pitch, PitchAcc, gyro_pitch_rate, MPU6050_SAMPLE_DT);
#else
    // 互补滤波模式
	RollGyro  = Roll + mpu6050_gyro_x * mpu6050_const_data2 * MPU6050_SAMPLE_DT;
	Roll      = 0.005 * RollAcc + (1 - 0.005) * RollGyro;
	
	PitchGyro = Pitch + mpu6050_gyro_y * mpu6050_const_data2 * MPU6050_SAMPLE_DT;
	Pitch     = 0.005 * PitchAcc + (1 - 0.005) * PitchGyro;
#endif
	
	// 偏航角计算：仅陀螺仪积分（无加速度计校准，会漂移）
	Yaw       += (float)mpu6050_gyro_z * mpu6050_const_data2 * MPU6050_SAMPLE_DT;
	
	// 一阶低通滤波
	Roll_Temp  = MPU6050_LOW_PASS_FILTER * Roll + (1 - MPU6050_LOW_PASS_FILTER) * Roll_Temp;
	Yaw_Temp   = MPU6050_LOW_PASS_FILTER * Yaw + (1 - MPU6050_LOW_PASS_FILTER) * Yaw_Temp;
	Pitch_Temp = MPU6050_LOW_PASS_FILTER * Pitch + (1 - MPU6050_LOW_PASS_FILTER) * Pitch_Temp;
	
	// 输出死区
	if ( fabs(Roll_Result-Roll_Temp  ) > MPU6050_OUTPUT_DEAD_ZONE ){Roll_Result = Roll_Temp;}
	if ( fabs(Yaw_Result-Yaw_Temp    ) > MPU6050_OUTPUT_DEAD_ZONE ){Yaw_Result = Yaw_Temp;}
	if ( fabs(Pitch_Result-Pitch_Temp) > MPU6050_OUTPUT_DEAD_ZONE ){Pitch_Result = Pitch_Temp;}
	
	#if USE_PITCH_AS_ANGLE
		Angle_Result = Pitch_Result + 3.75f;
	#else
		Angle_Result = Roll_Result;
	#endif
}

/*******************************************************************************************************************/
/*[S] 姿态解算 + 滤波 [S]------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
