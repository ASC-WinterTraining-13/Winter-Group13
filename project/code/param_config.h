#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

//pid结构体
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params_t;

extern PID_Params_t angle_pid;
extern PID_Params_t speed_pid;
extern PID_Params_t position_pid;
extern PID_Params_t turn_pid;


#endif