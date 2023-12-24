#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CAN_1 hcan1
#define CAN_2 hcan2

/* 云台电机 ID标识 */
#define CAN_GIMBAL_ALL_ID 0x2FE
#define CAN_YAW_MOTOR_ID 0x210 // attention：实则电机ID为6，该处为报文标识符

/* 摩擦轮电机 ID标识 */
#define CAN_FRIC_ALL_ID 0x200
#define CAN_FRIC_LEFT_ID 0x201
#define CAN_FRIC_RIGHT_ID 0x202

/* 推弹电机 ID标识 */
#define CAN_PUSH_MOTOR_ID 0x203
#define CAN_TURN_MOTOR_ID 0x209 // attention：实则电机ID为5，该处为报文标识符

// RoboMaster电机数据结构
typedef struct
{
    uint16_t ecd;          // 霍尔编码值机械角度
    int16_t speed_rpm;     // 转速
    int16_t given_current; // 电流
    uint8_t temperate;     // 温度
    int16_t last_ecd;      // 上一次机械角度
} motor_measure_t;

extern void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t push); // 发送发射机构电机控制电流CAN1

extern void CAN_cmd_gimbal(int16_t yaw, int16_t turn); // 发送yaw轴、转盘电机控制电流CAN2(0x205、0x206)

extern const motor_measure_t *get_right_fric_motor_measure_point(void); // 返回摩擦轮左电机 3508电机数据指针

extern const motor_measure_t *get_left_fric_motor_measure_point(void); // 返回摩擦轮右电机 3508电机数据指针

extern const motor_measure_t *get_yaw_motor_measure_point(void); // 返回yaw 6020电机数据指针

extern const motor_measure_t *get_turn_motor_measure_point(void); // 返回转盘电机 6020电机数据指针

extern const motor_measure_t *get_push_motor_measure_point(void); // 返回推弹电机 2006电机数据指针

#endif
