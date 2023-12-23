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
void yaw_control_loop(void);                                     // 电机控制

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
    
    /* 转盘PID初始化 */
    pid_init(&yaw_control_data.turn_position_pid, push_turn_position_pid, TURN_POSITION_MAX_OUT, TURN_POSITION_MAX_IOUT);
    pid_init(&yaw_control_data.turn_speed_pid, push_turn_speed_pid, TURN_SPEED_MAX_OUT, TURN_SPEED_MAX_IOUT);
    
    // 转盘电机数据指针绑定
    yaw_control_data.turn_motor_measure = get_turn_motor_measure_point();
    // 遥控器指针绑定
    yaw_control_data.yaw_rc = get_remote_control_point();
}

void yaw_feedback_update(yaw_control_data_t *yaw_feedback_update)
{
    // 角度当前位映射换算更新
    yaw_control_data.turn_motor_ref_angle = msp(yaw_control_data.turn_motor_measure->ecd, 0, 8191, -PI, PI);
}
void yaw_control_loop(void)
{
    // 内环（角度环）计算
    yaw_control_data.turn_inner_out = (int16_t)pid_calc(&yaw_control_data.turn_position_pid, yaw_control_data.turn_motor_ref_angle, yaw_control_data.turn_target_angle);
    // 外环（速度环）计算
    yaw_control_data.turn_motor_given_current = (int16_t)pid_calc(&yaw_control_data.turn_speed_pid, yaw_control_data.turn_motor_measure->speed_rpm, yaw_control_data.turn_inner_out);
}

void yaw_task()
{
    // 初始化
    yaw_init();
    //设定目标角度
    yaw_control_data.turn_target_angle = 1;

    while (1)
    {
        // 数据更新
        yaw_feedback_update(&yaw_control_data);
        //控制计算
        yaw_control_loop();
        // 发送电流
        CAN_cmd_gimbal(yaw_control_data.turn_motor_given_current,0);
        vTaskDelay(2);
    }
}
