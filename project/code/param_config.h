#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

//pid结构体
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params_t;

extern PID_Params_t angle_pid;
extern PID_Params_t mode_2_pid;
extern PID_Params_t mode_3_pid;
extern PID_Params_t mode_4_pid;
extern PID_Params_t mode_5_pid;


#endif