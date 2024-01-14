#include "shoot_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>
#include "bsp_rc.h"
#include "remote_control.h"

shoot_control_data_t shoot_control_data; // 发射机构全局数据

void shoot_init(void);                                                   // 发射机构初始化
void shoot_feedback_update(shoot_control_data_t *shoot_feedback_update); // 发射数据反馈更新
void shoot_mode_set(shoot_control_data_t *shoot_mode_set);               // 发射机构状态机设置
void shoot_control_loop(void);                                           // 发射控制

void shoot_init(void) // 发射机构初始化
{
    // 摩擦轮速度环PID参数
    const fp32 fric_left_speed_pid[3] = {FRIC_LEFT_SPEED_KP, FRIC_LEFT_SPEED_KI, FRIC_LEFT_SPEED_KD};
    const fp32 fric_right_speed_pid[3] = {FRIC_RIGHT_SPEED_KP, FRIC_RIGHT_SPEED_KI, FRIC_RIGHT_SPEED_KD};
    // 推杆电机PID参数
    const fp32 push_motor_speed_pid[3] = {PUSH_SPEED_KP, PUSH_SPEED_KI, PUSH_SPEED_KD};

    // 摩擦轮速度环PID初始化
    pid_init(&shoot_control_data.fric_left_pid, fric_left_speed_pid, FRIC_LEFT_SPEED_MAX_OUT, FRIC_LEFT_SPEED_MAX_IOUT);
    pid_init(&shoot_control_data.fric_right_pid, fric_right_speed_pid, FRIC_RIGHT_SPEED_MAX_OUT, FRIC_RIGHT_SPEED_MAX_IOUT);
    // 推杆速度环PID初始化
    pid_init(&shoot_control_data.push_motor_pid, push_motor_speed_pid, PUSH_SPEED_MAX_OUT, PUSH_SPEED_MAX_IOUT);

    // 摩擦轮电机数据指针绑定
    shoot_control_data.shoot_fric_left_motor = get_left_fric_motor_measure_point();
    shoot_control_data.shoot_fric_right_motor = get_right_fric_motor_measure_point();
    // 推杆电机数据指针绑定
    shoot_control_data.push_motor = get_push_motor_measure_point();

    // 状态初始化设定
    shoot_control_data.shoot_mode = FRIC_STOP;

    // 遥控器指针绑定
    shoot_control_data.shoot_rc = get_remote_control_point();
}

// 发射数据反馈更新
void shoot_feedback_update(shoot_control_data_t *shoot_feedback_update)
{
    // 摩擦轮线速度更新
    shoot_feedback_update->fric_left_ref_speed = shoot_control_data.shoot_fric_left_motor->speed_rpm * FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY;
    shoot_feedback_update->fric_right_ref_speed = shoot_control_data.shoot_fric_right_motor->speed_rpm * (-FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY);
    // 推杆电机转子速度更新
    shoot_feedback_update->push_motor_ref_speed = shoot_control_data.push_motor->speed_rpm;
    // 推杆--遥控器速度数据更新
    shoot_feedback_update->push_get_rc_speed = shoot_control_data.shoot_rc->rc.ch[3];
}

// 发射机构状态机设置
void shoot_mode_set(shoot_control_data_t *shoot_mode_set)
{
    // 左开关向上拨启动摩擦轮
    if (shoot_mode_set->shoot_mode == FRIC_STOP && shoot_control_data.shoot_rc->rc.s[1] == 0x01)
    {
        shoot_mode_set->shoot_mode = FRIC_RUN;
    }
    // 左开关中间档闭环停摩擦轮
    if (shoot_control_data.shoot_rc->rc.s[1] == 0x03)
    {
        shoot_mode_set->shoot_mode = FRIC_STOP;
    }
    // 左开关下拨发射机构无力
    if (shoot_control_data.shoot_rc->rc.s[1] == 0x02)
    {
        shoot_mode_set->shoot_mode = FRIC_NO_CURRENT;
    }
    // 挡位数据更新
    shoot_mode_set->last_switch = shoot_mode_set->shoot_rc->rc.s[1];
}
// 发射控制
void shoot_control_loop(void)
{
    // 发射机构状态机设置
    shoot_mode_set(&shoot_control_data);
    // 摩擦轮数据反馈更新
    shoot_feedback_update(&shoot_control_data);
    if (shoot_control_data.shoot_mode == FRIC_NO_CURRENT)
    {
        shoot_control_data.fric_left_given_current = 0;
        shoot_control_data.fric_right_given_current = 0;
    }
    else
    { /* 状态机为摩擦轮停止时，摩擦轮停止，推杆停止 */
        if (shoot_control_data.shoot_mode == FRIC_STOP)
        {
            shoot_control_data.fric_set_speed = FRIC_STOP_SPEED;
            shoot_control_data.push_set_speed = PUSH_STOP_SPEED;
        }
        /* 状态机为摩擦轮运行时，摩擦轮运行，左边4通道推杆控制2006速度 */
        if (shoot_control_data.shoot_mode == FRIC_RUN)
        {
            // 摩擦轮的速度设定
            shoot_control_data.fric_set_speed = FRIC_TARGGET_SPEED;
            // 推杆电机的速度设定
            shoot_control_data.push_set_speed = -(shoot_control_data.push_get_rc_speed * 6); // 推杆范围[-660,660],乘系数变推杆最大速度，M2006转速范围约[-8000,8000]
        }
        // 摩擦轮M3508闭环计算
        shoot_control_data.fric_left_given_current = (int16_t)pid_calc(&shoot_control_data.fric_left_pid, shoot_control_data.fric_left_ref_speed, shoot_control_data.fric_set_speed);
        shoot_control_data.fric_right_given_current = (int16_t)pid_calc(&shoot_control_data.fric_right_pid, shoot_control_data.fric_right_ref_speed, shoot_control_data.fric_set_speed);
        // 推杆M2006闭环计算
        shoot_control_data.push_motor_given_current = (int16_t)pid_calc(&shoot_control_data.push_motor_pid, shoot_control_data.push_motor_ref_speed, shoot_control_data.push_set_speed);
    }
}
// task主函数
void shoot_task()
{

    shoot_init(); // 初始化发射机构
    while (1)
    {
        // 摩擦轮闭环
        shoot_control_loop();
        // 发送电流
        CAN_cmd_shoot(shoot_control_data.fric_left_given_current, shoot_control_data.fric_right_given_current, shoot_control_data.push_motor_given_current);
        // 等待接收数据刷新，避免刷新速度过快
        vTaskDelay(2);
    }
}
