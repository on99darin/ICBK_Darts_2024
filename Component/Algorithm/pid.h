#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "struct_typedef.h"
#include "user_lib.h"

typedef struct
{
    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次

} pid_data_t;

extern void pid_init(pid_data_t *pid, const float PID[3], float max_out, float max_iout);
extern float pid_calc(pid_data_t *pid, float ref, float set);

#endif
