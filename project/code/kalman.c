#include "zf_common_headfile.h"
#include "kalman.h"  // 必须先包含头文件，才能使用结构体

#define delta_T     0.0025f  // 400Hz更新频率（2.5ms）
float I_ex, I_ey, I_ez;

// 全局变量初始化（现在能识别结构体类型）
quater_param_t Q_info = {1, 0, 0, 0};
euler_param_t eulerAngle;
icm_param_t icm_data;
gyro_param_t GyroOffset;

bool GyroOffset_init = 0;
float param_Kp = 0.05;
float param_Ki = 0.2;
float angle_Z = 0,angle_R = 0,angle_P = 0;

// 快速开方（优化版）
float fast_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 陀螺仪零偏校准（适配逐飞IMU660RA接口）
void gyroOffset_init(void)
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    
    // 采样100次求平均（逐飞标准IMU接口）
    for (uint16_t i = 0; i < 100; ++i) {
        imu660ra_get_acc();
        imu660ra_get_gyro();
        GyroOffset.Xdata += imu660ra_gyro_x;
        GyroOffset.Ydata += imu660ra_gyro_y;
        GyroOffset.Zdata += imu660ra_gyro_z;
        system_delay_ms(10);
    }
    
    GyroOffset.Xdata /= 100;
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;
    eulerAngle.Dirchange=0;
    GyroOffset_init = 1;
}

#define alpha           0.4f  // 低通滤波系数

// 获取IMU数据（低通滤波+零偏校准）
void ICM_getValues() {
    // 加速度计低通滤波
    icm_data.acc_x = (((float) imu660ra_acc_x) * alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float) imu660ra_acc_y) * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float) imu660ra_acc_z) * alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);

    // 陀螺仪零偏校准+单位转换（°/s → rad/s）
    icm_data.gyro_x = ((float) imu660ra_gyro_x - GyroOffset.Xdata) * PI / 180 / 16.4f;
    icm_data.gyro_y = ((float) imu660ra_gyro_y - GyroOffset.Ydata) * PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) imu660ra_gyro_z - GyroOffset.Zdata) * PI / 180 / 16.4f;
}

// 四元数更新（AHRS姿态解算）
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0.17;

    // 加速度计数据归一化
    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;  // 修正：原代码是ax=ax*norm，除法才是归一化
    ay = ay / norm;
    az = az / norm;

    // 四元数转换为重力向量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 计算误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 积分误差
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // 比例+积分校正陀螺仪
    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    // 四元数更新（一阶龙格库塔）
    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 四元数归一化
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 / norm;
    Q_info.q1 = q1 / norm;
    Q_info.q2 = q2 / norm;
    Q_info.q3 = q3 / norm;
}

// 四元数转欧拉角
void ICM_getEulerianAngles(void) {
    static uint8 change_f=0;
    ICM_getValues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    // 四元数转欧拉角（适配Keil V6，asin/atan2已包含math.h）
    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI;
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI;
    eulerAngle.yaw =  atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180/ PI;

    // 偏航角范围修正（-180~180）
    if (eulerAngle.yaw >= 180) { eulerAngle.yaw -= 360; } 
    else if (eulerAngle.yaw <= -180) { eulerAngle.yaw += 360; }

    // 偏航角跨0度处理
    if((eulerAngle.yaw-eulerAngle.last_yaw) < -350) eulerAngle.Dirchange++;
    else if ((eulerAngle.yaw-eulerAngle.last_yaw) > 350) eulerAngle.Dirchange--;

    // 累计偏航角（解决360度循环问题）
    angle_Z=360*eulerAngle.Dirchange+eulerAngle.yaw;
    eulerAngle.last_yaw=eulerAngle.yaw;
}
