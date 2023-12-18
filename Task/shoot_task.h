#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__
#endif

#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_rc.h"
#include "remote_control.h"

#define FRIC_STOP_SPEED 0.0f
#define FRIC_TARGGET_SPEED 0.20f // 设定摩擦轮的线速度 m/s

#define FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY 0.00005f // 摩擦轮角速度到线速度的转换率 = 摩擦轮半径(m) / 60;

/* 左摩擦轮电机速度PID */
#define FRIC_LEFT_SPEED_KP 5000.0f
#define FRIC_LEFT_SPEED_KI 30.0f
#define FRIC_LEFT_SPEED_KD 10.0f
#define FRIC_LEFT_SPEED_PID_MAX_OUT 20000.0f
#define FRIC_LEFT_SPEED_PID_MAX_IOUT 2500.0f

/* 右边摩擦轮电机速度PID */
#define FRIC_RIGHT_SPEED_KP 5000.0f
#define FRIC_RIGHT_SPEED_KI 30.0f
#define FRIC_RIGHT_SPEED_KD 10.0f
#define FRIC_RIGHT_SPEED_PID_MAX_OUT 20000.0f
#define FRIC_RIGHT_SPEED_PID_MAX_IOUT 2500.0f

void shoot_task(void);

// 状态模式列表
typedef enum
{
    FRIC_STOP,      // 摩擦轮停止状态
    FRIC_RUN,       // 摩擦轮开启状态
    FRIC_NO_CURRENT // 摩擦轮无力状态
} shoot_control_mode_e;

// 发射机构数据结构体
typedef struct
{
    shoot_control_mode_e shoot_mode; // 发射机构状态机

    const RC_ctrl_t *shoot_rc; // 遥控器数据

    const motor_measure_t *shoot_fric_left_motor;  // 左摩擦轮电机数据
    const motor_measure_t *shoot_fric_right_motor; // 右摩擦轮电机数据

    pid_data_t fric_left_pid;  // 左摩擦轮速度环PID
    pid_data_t fric_right_pid; // 右摩擦轮速度环PID

    fp32 fric_set_speed;       // 摩擦轮预定线速度
    fp32 fric_left_ref_speed;  // 左摩擦轮反馈的线速度
    fp32 fric_right_ref_speed; // 右摩擦轮反馈的线速度

    int16_t fric_left_given_current;  // 给定左摩擦轮电机的电流值
    int16_t fric_right_given_current; // 给定右摩擦轮电机的电流值

    char last_switch; // 上一次的挡位

} shoot_control_data_t;
