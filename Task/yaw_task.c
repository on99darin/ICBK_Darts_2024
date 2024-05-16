/**
  * @file       yaw_task.c/h
  * @brief      飞镖yaw轴任务，yaw电机
  * @note
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  yaw和turn电机初始位在.c文件修改。并不在.h文件
  ==============================================================================
  */
#include "yaw_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>
#include "bsp_rc.h"
#include "remote_control.h"
#include "shoot_task.h"

yaw_control_data_t yaw_control_data;                               // 全局数据
void yaw_init(void);                                               // yaw与turn电机初始化
void yaw_feedback_update(yaw_control_data_t *yaw_feedback_update); // yaw数据反馈更新
void yaw_control_loop(void);                                       // 电机控制
void yaw_mode_set(yaw_control_data_t *yaw_mode_set);               // 状态机刷新

void yaw_init(void)
{
    // 等待系统上线
    vTaskDelay(10);
    // yaw位置环PID参数
    const fp32 gimbal_yaw_position_pid[3] = {YAW_POSITION_KP, YAW_POSITION_KI, YAW_POSITION_KD};
    // yaw速度环PID参数
    const fp32 gimbal_yaw_speed_pid[3] = {YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD};

    /* YAW电机PID初始化 */
    // yaw位置环PID初始化
    pid_init(&yaw_control_data.yaw_position_pid, gimbal_yaw_position_pid, YAW_POSITION_MAX_OUT, YAW_POSITION_MAX_IOUT);
    // yaw速度环PID初始化
    pid_init(&yaw_control_data.yaw_speed_pid, gimbal_yaw_speed_pid, YAW_SPEED_MAX_OUT, YAW_SPEED_MAX_IOUT);
    // YAW电机数据指针绑定
    yaw_control_data.yaw_motor_measure = get_yaw_motor_measure_point();
    // 遥控器指针绑定
    yaw_control_data.yaw_rc = get_remote_control_point();
    // YAW电机初始位
    yaw_control_data.yaw_target_angle = 3825.0f;
    // 状态机初始位
    yaw_control_data.yaw_mode = YAW_LOCK;
}

void yaw_feedback_update(yaw_control_data_t *yaw_feedback_update)
{
    // YAW电机使用ecd[0-8191]
    yaw_control_data.yaw_motor_ref_angle = yaw_control_data.yaw_motor_measure->ecd;
    // YAW电机--遥控器速度数据更新
    yaw_control_data.yaw_get_rc_add_ecd = yaw_control_data.yaw_rc->rc.ch[0];
}

void yaw_control_loop(void)
{
    // 左下解锁无力状态
    if (yaw_control_data.yaw_mode == YAW_UNLOCK)
    {
        yaw_control_data.yaw_target_angle += (yaw_control_data.yaw_get_rc_add_ecd * RC_TO_YAW);
        // yaw轴电机限幅
        if (yaw_control_data.yaw_target_angle > 5250)
        {
            yaw_control_data.yaw_target_angle = 5250;
        }
        else if (yaw_control_data.yaw_target_angle < 2050)
        {
            yaw_control_data.yaw_target_angle = 2050;
        }
    }
    else
    {
        // 更新目标角度，平衡对角发射
        if (yaw_control_data.yaw_mode == TURN_GO)
        {
            switch (yaw_control_data.turn_motor_time)
            {
            case 0:
            {
                yaw_control_data.turn_target_angle += PI;
                yaw_control_data.turn_motor_time = 1;
                break;
            }

            case 1:
            {
                yaw_control_data.turn_target_angle += PI / 2;
                yaw_control_data.turn_motor_time = 2;
                break;
            }
            case 2:
            {
                yaw_control_data.turn_target_angle -= PI;
                yaw_control_data.turn_motor_time = 3;
                break;
            }
            case 3:
            {
                yaw_control_data.turn_target_angle = TURN_INIT_ANGLE;
                yaw_control_data.turn_motor_time = 0;
                break;
            }
            default:
            {
                break;
            }
            }
            yaw_control_data.yaw_mode = TURN_OVER;
        }
        // yaw角度环计算
        yaw_control_data.yaw_inner_out = (int16_t)pid_calc(&yaw_control_data.yaw_position_pid, yaw_control_data.yaw_motor_ref_angle, yaw_control_data.yaw_target_angle);
        // yaw速度环计算
        yaw_control_data.yaw_motor_given_current = (int16_t)pid_calc(&yaw_control_data.yaw_speed_pid, yaw_control_data.yaw_motor_measure->speed_rpm, yaw_control_data.yaw_inner_out);
        CAN_cmd_yaw(yaw_control_data.yaw_motor_given_current);
    }
}

void yaw_mode_set(yaw_control_data_t *yaw_mode_set)
{
    if (yaw_mode_set->yaw_mode == YAW_UNLOCK && yaw_control_data.yaw_rc->rc.s[0] == 0x01)
    {
        yaw_mode_set->yaw_mode = YAW_LOCK;
    }
}

// 主任务
void yaw_task()
{
    // 初始化
    yaw_init();

    while (1)
    {
        // 数据更新
        yaw_feedback_update(&yaw_control_data);
        // 控制计算
        yaw_control_loop();
        // 设置更新状态机
        yaw_mode_set(&yaw_control_data);
        // 避免刷新过快
        vTaskDelay(1);
    }
}
