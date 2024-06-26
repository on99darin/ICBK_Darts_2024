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

/* yaw速度环PID */
#define YAW_SPEED_KP 10.0f
#define YAW_SPEED_KI 0.01f
#define YAW_SPEED_KD 0.0f
#define YAW_SPEED_MAX_OUT 10000.0f
#define YAW_SPEED_MAX_IOUT 2000.0f

/* yaw角度环PID */
#define YAW_POSITION_KP 5.0f
#define YAW_POSITION_KI 0.0f
#define YAW_POSITION_KD 0.1f
#define YAW_POSITION_MAX_OUT 500.0f
#define YAW_POSITION_MAX_IOUT 80.0f

// YAW初始位（ECD）
#define YAW_INIT_ANGLE 10.0f

// yaw轴遥控器数据转ecd增量
#define RC_TO_YAW 0.0006f

/* yaw电机限位置 */
#define YAW_LIMIT_MAX_ECD 3000
#define YAW_LIMIT_MIN_ECD 6000
// 状态模式列表
typedef enum
{
    YAW_LOCK,
    YAW_UNLOCK
}yaw_control_mode_e;

typedef struct
{
    const RC_ctrl_t *yaw_rc;     // 遥控器数据
    yaw_control_mode_e yaw_mode; // 状态机

    const motor_measure_t *yaw_motor_measure; // YAW电机数据
    pid_data_t yaw_position_pid;              // YAW角度环PID
    pid_data_t yaw_speed_pid;                 // YAW速度环PID

    fp32 yaw_motor_ref_angle; // YAW电机反馈换算后的角度

    fp32 yaw_inner_out; // YAW内环（角度环）PID计算值

    int16_t yaw_motor_given_current; // 给定YAW电机的电流值

    fp32 yaw_target_angle; // YAW目标角度

    int16_t yaw_get_rc_add_ecd; // 存放遥控器给YAW速度的数据

} yaw_control_data_t;

#endif
