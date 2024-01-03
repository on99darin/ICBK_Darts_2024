#include "yaw_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>
#include "bsp_rc.h"
#include "remote_control.h"

yaw_control_data_t yaw_control_data;                               // 全局数据
void yaw_init(void);                                               // yaw与turn电机初始化
void yaw_feedback_update(yaw_control_data_t *yaw_feedback_update); // yaw数据反馈更新
void yaw_control_loop(void);                                       // 电机控制

// 映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-PI~PI）
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void yaw_init(void)
{
    // 转盘位置环PID参数
    const fp32 push_turn_position_pid[3] = {TURN_POSITION_KP, TURN_POSITION_KI, TURN_POSITION_KD};
    // 转盘速度环PID参数
    const fp32 push_turn_speed_pid[3] = {TURN_SPEED_KP, TURN_SPEED_KI, TURN_SPEED_KD};
    // yaw位置环PID参数
    const fp32 gimbal_yaw_position_pid[3] = {YAW_POSITION_KP, YAW_POSITION_KI, YAW_POSITION_KD};
    // yaw速度环PID参数
    const fp32 gimbal_yaw_speed_pid[3] = {YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD};

    /* 转盘PID初始化 */
    // 转盘位置环PID初始化
    pid_init(&yaw_control_data.turn_position_pid, push_turn_position_pid, TURN_POSITION_MAX_OUT, TURN_POSITION_MAX_IOUT);
    // 转盘速度环PID初始化
    pid_init(&yaw_control_data.turn_speed_pid, push_turn_speed_pid, TURN_SPEED_MAX_OUT, TURN_SPEED_MAX_IOUT);

    /* 转盘PID初始化 */
    // yaw位置环PID初始化
    pid_init(&yaw_control_data.yaw_position_pid, gimbal_yaw_position_pid, YAW_POSITION_MAX_OUT, YAW_POSITION_MAX_IOUT);
    // yaw速度环PID初始化
    pid_init(&yaw_control_data.yaw_speed_pid, gimbal_yaw_speed_pid, YAW_SPEED_MAX_OUT, YAW_SPEED_MAX_IOUT);

    // 转盘电机数据指针绑定
    yaw_control_data.turn_motor_measure = get_turn_motor_measure_point();
    // YAW电机数据指针绑定
    yaw_control_data.yaw_motor_measure = get_yaw_motor_measure_point();
    // 遥控器指针绑定
    yaw_control_data.yaw_rc = get_remote_control_point();
    // 转盘电机初始位
    yaw_control_data.turn_target_angle = 1.06854808f;
    // 状态机初始位
    yaw_control_data.yaw_mode = TURN_READY;
}

void yaw_feedback_update(yaw_control_data_t *yaw_feedback_update)
{
    // 角度当前位映射换算更新
    yaw_control_data.turn_motor_ref_angle = msp(yaw_control_data.turn_motor_measure->ecd, 0, 8191, 0, 2 * PI);
    yaw_control_data.yaw_motor_ref_angle = msp(yaw_control_data.yaw_motor_measure->ecd, 0, 8191, 0, 2 * PI);
}

void yaw_control_loop(void)
{
    // 更新目标角度
    if (yaw_control_data.yaw_mode == TURN_GO)
    {
        yaw_control_data.turn_target_angle += PI / 2;
        if (yaw_control_data.turn_target_angle > 2 * PI)
        {
            yaw_control_data.turn_target_angle = 1.06854808f;
        }
        yaw_control_data.yaw_mode = TURN_OVER;
    }
    // 转盘角度环计算
    yaw_control_data.turn_inner_out = (int16_t)pid_calc(&yaw_control_data.turn_position_pid, yaw_control_data.turn_motor_ref_angle, yaw_control_data.turn_target_angle);
    // 转盘速度环计算
    yaw_control_data.turn_motor_given_current = (int16_t)pid_calc(&yaw_control_data.turn_speed_pid, yaw_control_data.turn_motor_measure->speed_rpm, yaw_control_data.turn_inner_out);
    // yaw角度环计算
    yaw_control_data.yaw_inner_out = (int16_t)pid_calc(&yaw_control_data.yaw_position_pid, yaw_control_data.yaw_motor_ref_angle, yaw_control_data.yaw_target_angle);
    // yaw速度环计算
    yaw_control_data.yaw_motor_given_current = (int16_t)pid_calc(&yaw_control_data.yaw_speed_pid, yaw_control_data.yaw_motor_measure->speed_rpm, yaw_control_data.yaw_inner_out);
}

void yaw_mode_set(yaw_control_data_t *yaw_mode_set)
{
    // 右开关向上拨转一圈
    if (yaw_mode_set->yaw_mode == TURN_READY && yaw_control_data.yaw_rc->rc.s[0] == 0x01)
    {
        yaw_mode_set->yaw_mode = TURN_GO;
    }
    // 右开关向上拨转一圈
    if (yaw_mode_set->yaw_mode == TURN_OVER && yaw_control_data.yaw_rc->rc.s[0] == 0x03)
    {
        yaw_mode_set->yaw_mode = TURN_READY;
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
        // 发送电流
        CAN_cmd_gimbal(yaw_control_data.turn_motor_given_current, yaw_control_data.yaw_motor_given_current);
        vTaskDelay(2);
    }
}
