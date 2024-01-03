#ifndef YAW_TASK_H
#define YAW_TASK_H

#include "yaw_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>
#include "bsp_rc.h"
#include "remote_control.h"

#define PI 3.14159265358979f

/* 转盘速度环PID */
#define TURN_SPEED_KP 60.0f
#define TURN_SPEED_KI 0.0f
#define TURN_SPEED_KD 0.0f
#define TURN_SPEED_MAX_OUT 20000.0f
#define TURN_SPEED_MAX_IOUT 3000.0f

/* 转盘角度环PID */
#define TURN_POSITION_KP 500.0f
#define TURN_POSITION_KI 0.0f
#define TURN_POSITION_KD 0.0f
#define TURN_POSITION_MAX_OUT 320.0f
#define TURN_POSITION_MAX_IOUT 0.0f

/* 转盘初始位ECD */
#define TURN_INIT_ECD 0

/* yaw速度环PID */
#define YAW_SPEED_KP 0.0f
#define YAW_SPEED_KI 0.0f
#define YAW_SPEED_KD 0.0f
#define YAW_SPEED_MAX_OUT 0.0f
#define YAW_SPEED_MAX_IOUT 0.0f

/* yaw角度环PID */
#define YAW_POSITION_KP 0.0f
#define YAW_POSITION_KI 0.0f
#define YAW_POSITION_KD 0.0f
#define YAW_POSITION_MAX_OUT 0.0f
#define YAW_POSITION_MAX_IOUT 0.0f

// 状态模式列表
typedef enum
{
    TURN_READY, // 准备转状态
    TURN_GO,    // 开转
    TURN_OVER,  // 转完
    YAW_UNLOCK  // 解锁YAW轴
} yaw_control_mode_e;

typedef struct
{
    const RC_ctrl_t *yaw_rc;     // 遥控器数据
    yaw_control_mode_e yaw_mode; // 状态机

    const motor_measure_t *turn_motor_measure; // 转盘电机数据
    const motor_measure_t *yaw_motor_measure;  // YAW电机数据

    pid_data_t turn_position_pid; // 转盘角度环PID
    pid_data_t turn_speed_pid;    // 转盘速度环PID
    pid_data_t yaw_position_pid;  // YAW角度环PID
    pid_data_t yaw_speed_pid;     // YAW速度环PID

    fp32 turn_motor_ref_angle; // 转盘电机反馈换算后的角度
    fp32 yaw_motor_ref_angle;  // YAW电机反馈换算后的角度

    fp32 turn_inner_out; // 转盘内环（角度环）PID计算值
    fp32 yaw_inner_out;  // YAW内环（角度环）PID计算值

    int16_t turn_motor_given_current; // 给定转盘电机的电流值
    int16_t yaw_motor_given_current;  // 给定转盘电机的电流值

    fp32 turn_target_angle; // TURN目标角度
    fp32 yaw_target_angle;  // YAW目标角度

} yaw_control_data_t;

#endif
