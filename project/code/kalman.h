#ifndef __KALMAN_H_
#define __KALMAN_H_

#include "zf_common_typedef.h"
#include <math.h>  // 必须包含，解决asin/atan2报错

// IMU数据结构体
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;

// 四元数结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

// 欧拉角结构体
typedef struct {
    float pitch;    // 俯仰角
    float roll;     // 横滚角
    float yaw;      // 偏航角
    float last_yaw; // 上一帧偏航角
    int8 Dirchange; // 方向切换计数
} euler_param_t;

// 陀螺仪零偏结构体
typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

// 全局变量（提前声明，避免编译顺序问题）
extern quater_param_t Q_info;
extern euler_param_t eulerAngle;
extern icm_param_t icm_data;
extern gyro_param_t GyroOffset;
extern float angle_Z,angle_R,angle_P;
extern bool GyroOffset_init;

// 函数声明
void gyroOffset_init(void);
float fast_sqrt(float x);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getValues();
void ICM_getEulerianAngles(void);

#endif
