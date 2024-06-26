/**
  * @file       shoot_task.c/h
  * @brief      飞镖发射任务，摩擦轮x2+推杆电机
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0                       DARIN
  @verbatim
  ==============================================================================
  */
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
void turn_mode_set(shoot_control_data_t *turn_mode_set);
void shoot_control_loop(void); // 发射控制
void push_limit_control(void);

// 映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-PI~PI）
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief          发射机构初始化
 * @param[in]      none
 * @retval         none
 */
void shoot_init(void)
{
    // 摩擦轮速度环PID参数
    const fp32 fric_left_speed_pid[3] = {FRIC_LEFT_SPEED_KP, FRIC_LEFT_SPEED_KI, FRIC_LEFT_SPEED_KD};
    const fp32 fric_right_speed_pid[3] = {FRIC_RIGHT_SPEED_KP, FRIC_RIGHT_SPEED_KI, FRIC_RIGHT_SPEED_KD};
    // 推杆电机PID参数
    const fp32 push_motor_speed_pid[3] = {PUSH_SPEED_KP, PUSH_SPEED_KI, PUSH_SPEED_KD};
    // 转盘位置环PID参数
    const fp32 push_turn_position_pid[3] = {TURN_POSITION_KP, TURN_POSITION_KI, TURN_POSITION_KD};
    // 转盘速度环PID参数
    const fp32 push_turn_speed_pid[3] = {TURN_SPEED_KP, TURN_SPEED_KI, TURN_SPEED_KD};
    // 摩擦轮速度环PID初始化
    pid_init(&shoot_control_data.fric_left_pid, fric_left_speed_pid, FRIC_LEFT_SPEED_MAX_OUT, FRIC_LEFT_SPEED_MAX_IOUT);
    pid_init(&shoot_control_data.fric_right_pid, fric_right_speed_pid, FRIC_RIGHT_SPEED_MAX_OUT, FRIC_RIGHT_SPEED_MAX_IOUT);
    // 推杆速度环PID初始化
    pid_init(&shoot_control_data.push_motor_pid, push_motor_speed_pid, PUSH_SPEED_MAX_OUT, PUSH_SPEED_MAX_IOUT);
    /*转盘电机PID初始化*/
    // 转盘位置环PID初始化
    pid_init(&shoot_control_data.turn_position_pid, push_turn_position_pid, TURN_POSITION_MAX_OUT, TURN_POSITION_MAX_IOUT);
    // 转盘速度环PID初始化
    pid_init(&shoot_control_data.turn_speed_pid, push_turn_speed_pid, TURN_SPEED_MAX_OUT, TURN_SPEED_MAX_IOUT);
    // 摩擦轮电机数据指针绑定
    shoot_control_data.shoot_fric_left_motor = get_left_fric_motor_measure_point();
    shoot_control_data.shoot_fric_right_motor = get_right_fric_motor_measure_point();
    // 推杆电机数据指针绑定
    shoot_control_data.push_motor = get_push_motor_measure_point();
    // 转盘电机数据指针绑定
    shoot_control_data.turn_motor_measure = get_turn_motor_measure_point();
    // 遥控器指针绑定
    shoot_control_data.shoot_rc = get_remote_control_point();
    // 摩擦轮状态初始化设定
    shoot_control_data.shoot_mode = FRIC_STOP;
    // 转盘推杆状态初始化设定
    shoot_control_data.turn_mode = SHOOT_WAITE;
    // 转盘电机初始位
    shoot_control_data.turn_target_angle = TURN_INIT_ANGLE;
    // TURN电机发射次数初始化
    shoot_control_data.turn_motor_time = 0;
    // PUSH电机微动限位扫描
    push_limit_control();
}

/**
 * @brief          push电机推动扫描限位
 * @param[in]      none
 * @retval         none
 */
void push_limit_control(void)
{
    shoot_control_data.push_up_flag = HAL_GPIO_ReadPin(UP_DETECT_GPIO_Port, UP_DETECT_Pin);
    shoot_control_data.push_down_flag = HAL_GPIO_ReadPin(DOWN_DETECT_GPIO_Port, DOWN_DETECT_Pin);
}

/**
 * @brief          发射数据反馈更新
 * @param[in]      none
 * @retval         发射数据指针
 */
void shoot_feedback_update(shoot_control_data_t *shoot_feedback_update)
{
    // 摩擦轮线速度更新
    shoot_feedback_update->fric_left_ref_speed = shoot_control_data.shoot_fric_left_motor->speed_rpm * FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY;
    shoot_feedback_update->fric_right_ref_speed = shoot_control_data.shoot_fric_right_motor->speed_rpm * (-FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY);
    // 推杆电机转子速度更新
    shoot_feedback_update->push_motor_ref_speed = shoot_control_data.push_motor->speed_rpm;
    // TURN电机使用PI[0-2*PI]
    shoot_feedback_update->turn_motor_ref_angle = msp(shoot_control_data.turn_motor_measure->ecd, 0, 8191, 0, 2 * PI);
    // 推杆--遥控器速度数据更新
    shoot_feedback_update->push_get_rc_speed = shoot_control_data.shoot_rc->rc.ch[3];
    // PUSH电机微动限位扫描
    push_limit_control();
}

/**
 * @brief          发射机构状态机设置
 * @param[in]      none
 * @retval         发射状态机模式
 */
void shoot_mode_set(shoot_control_data_t *shoot_mode_set)
{
    if (shoot_control_data.shoot_rc->rc.s[0] == 0x02)
    {
        shoot_control_data.darts_mode_set = 0; // 0为裁判主控模式
    }
    else
    {
        shoot_control_data.darts_mode_set = 1; // 1为遥控模式
    }

    if (shoot_control_data.darts_mode_set == 1)
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
            shoot_mode_set->shoot_mode = SHOOT_NO_CURRENT;
        }
    }
}
void turn_mode_set(shoot_control_data_t *turn_mode_set)
{
    if (shoot_control_data.darts_mode_set == 1)
    {
        if (shoot_control_data.turn_mode == SHOOT_WAITE && shoot_control_data.shoot_rc->rc.s[0] == 0x01)
        {
            shoot_control_data.turn_mode = SHOOT_READY;
        }
    }
}

/**
 * @brief          发射控制
 * @param[in]      none
 * @retval         none
 */
void shoot_control_loop(void)
{
    // 发射机构状态机设置
    shoot_mode_set(&shoot_control_data);
	turn_mode_set(&shoot_control_data);
    // 摩擦轮数据反馈更新
    shoot_feedback_update(&shoot_control_data);
    // 判断状态机是否为无力状态
    if (shoot_control_data.shoot_mode == SHOOT_NO_CURRENT)
    {
        // 摩擦轮发送电流为0
        shoot_control_data.fric_left_given_current = 0;
        shoot_control_data.fric_right_given_current = 0;
        // 推弹电机发射电流为0
        shoot_control_data.push_motor_given_current = 0;
        // 转盘电机发射电流为0
        shoot_control_data.turn_motor_given_current = 0;
    }
    else
    // 如果状态机不为无力状态，进行下面的判断
    {
        /* 状态机为摩擦轮停止时，摩擦轮停止，推杆停止 */
        if (shoot_control_data.shoot_mode == FRIC_STOP)
        {
            // 此时摩擦轮速度为0，不允许用手拨动摩擦轮
            shoot_control_data.fric_set_speed = FRIC_STOP_SPEED;
            shoot_control_data.push_set_speed = PUSH_STOP_SPEED;
        }
        /* 状态机为摩擦轮运行时，摩擦轮运行，左边4通道推杆控制3508速度 */
        if (shoot_control_data.shoot_mode == FRIC_RUN)
        {
            // 摩擦轮的速度设定
            shoot_control_data.fric_set_speed = FRIC_TARGGET_SPEED;
            // 遥控器控制自动发射信号
            if (shoot_control_data.turn_mode == SHOOT_READY)
            {
                shoot_control_data.push_set_speed = -10000;
                shoot_control_data.turn_mode = PUSH_UP;
            }
            if (shoot_control_data.turn_mode == PUSH_UP)
            {
                if (shoot_control_data.push_up_flag == 0)
                {
                    shoot_control_data.push_set_speed = 0;
                    shoot_control_data.turn_mode = PUSH_DOWN_READY;
                }
            }
            if (shoot_control_data.turn_mode == PUSH_DOWN_READY)
            {
                shoot_control_data.push_set_speed = 12000;
                shoot_control_data.turn_mode = PUSH_DOWN;
            }
            if (shoot_control_data.turn_mode == PUSH_DOWN)
            {
                if (shoot_control_data.push_down_flag == 0)
                {
                    shoot_control_data.push_set_speed = 0;
                    shoot_control_data.turn_mode = SHOOT_OVER;
                }
            }
            if (shoot_control_data.turn_mode == SHOOT_OVER)
            {
                shoot_control_data.turn_mode = TURN_GO;
            }
            // 更新目标角度，平衡对角发射
            if (shoot_control_data.turn_mode == TURN_GO)
            {
                switch (shoot_control_data.turn_motor_time)
                {
                case 0:
                {
                    shoot_control_data.turn_target_angle += PI;
                    shoot_control_data.turn_motor_time = 1;
                    break;
                }

                case 1:
                {
                    shoot_control_data.turn_target_angle += PI / 2;
                    shoot_control_data.turn_motor_time = 2;
                    break;
                }
                case 2:
                {
                    shoot_control_data.turn_target_angle -= PI;
                    shoot_control_data.turn_motor_time = 3;
                    break;
                }
                case 3:
                {
                    shoot_control_data.turn_target_angle = TURN_INIT_ANGLE;
                    shoot_control_data.turn_motor_time = 0;
                    break;
                }
                default:
                {
                    break;
                }
                }
                shoot_control_data.turn_mode = TURN_OVER;
            }
            if (shoot_control_data.turn_mode == TURN_OVER)
            {
                shoot_control_data.turn_mode = SHOOT_WAITE;
            }
            /*调试速度
            // 推杆电机的速度设定
            shoot_control_data.push_set_speed = -(shoot_control_data.push_get_rc_speed * 15);
            // 下微动开关限位
            if (shoot_control_data.push_up_flag == 0 && shoot_control_data.push_set_speed > 0)
            {
                shoot_control_data.push_set_speed = PUSH_STOP_SPEED;
            }
            // 上微动开关限位
            if (shoot_control_data.push_down_flag == 0 && shoot_control_data.push_set_speed < 0)
            {
                shoot_control_data.push_set_speed = PUSH_STOP_SPEED;
            }
            */
        }
        // 摩擦轮M3508闭环计算
        shoot_control_data.fric_left_given_current = (int16_t)pid_calc(&shoot_control_data.fric_left_pid, shoot_control_data.fric_left_ref_speed, shoot_control_data.fric_set_speed);
        shoot_control_data.fric_right_given_current = (int16_t)pid_calc(&shoot_control_data.fric_right_pid, shoot_control_data.fric_right_ref_speed, shoot_control_data.fric_set_speed);
        // 推杆M3508闭环计算
        shoot_control_data.push_motor_given_current = (int16_t)pid_calc(&shoot_control_data.push_motor_pid, shoot_control_data.push_motor_ref_speed, shoot_control_data.push_set_speed);
        // 转盘角度环计算
        shoot_control_data.turn_inner_out = (int16_t)pid_calc(&shoot_control_data.turn_position_pid, shoot_control_data.turn_motor_ref_angle, shoot_control_data.turn_target_angle);
        // 转盘速度环计算
        shoot_control_data.turn_motor_given_current = (int16_t)pid_calc(&shoot_control_data.turn_speed_pid, shoot_control_data.turn_motor_measure->speed_rpm, shoot_control_data.turn_inner_out);
    }
}

/**
 * @brief          task主函数
 * @param[in]      none
 * @retval         none
 */
void shoot_task(void const *argument)
{

    shoot_init(); // 初始化发射机构
    while (1)
    {
        // 发射控制刷新
        shoot_control_loop();
        // 发送电流
        CAN_cmd_shoot(shoot_control_data.fric_left_given_current, shoot_control_data.fric_right_given_current,shoot_control_data.push_motor_given_current);
		//CAN_cmd_turn(shoot_control_data.turn_motor_given_current);
        //  等待接收数据刷新，避免刷新速度过快
        vTaskDelay(1);
    }
}
