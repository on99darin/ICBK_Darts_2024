#include "CAN_receive.h"
#include "main.h"


/*
    规定:
        CAN1负责发射任务，CAN1:左摩擦轮为201,右摩擦轮为202
        CAN1负责推镖任务，CAN1：转盘为203，推镖为204
        CAN2负责云台任务,CAN2:yaw为205

*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

motor_measure_t motor_push_data[2];   // 底盘四电机 M3508 M2006
motor_measure_t motor_gimbal_data[1]; // 云台一电机 YAW
motor_measure_t motor_fric_data[2];   // 摩擦轮二电机 LEFT RIGHT

static CAN_TxHeaderTypeDef can1_tx_message;
static CAN_TxHeaderTypeDef can2_tx_message;
static uint8_t can1_send_data[8];
static uint8_t can2_send_data[8];

void CAN_to_sbus(CAN_RxHeaderTypeDef rx_header, uint8_t rx_data[8]); // 处理遥控器数据

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN的接收回调回调函数
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) // CAN1频道
    {
        switch (rx_header.StdId)
        {

        case CAN_FRIC_LEFT_ID:
        {
            get_motor_measure(&motor_fric_data[0], rx_data);
            break;
        }

        case CAN_FRIC_RIGHT_ID:
        {
            get_motor_measure(&motor_fric_data[1], rx_data);
            break;
        }

        case CAN_PUSH_M3508_ID:
        {
            get_motor_measure(&motor_push_data[0], rx_data);
            break;
        }

        case CAN_PUSH_M2006_ID:
        {
            get_motor_measure(&motor_push_data[1], rx_data);
            break;
        }

        default:
        {
            break;
        }
        }
    }

    else if (hcan == &hcan2) // CAN2频道
    {
        switch (rx_header.StdId)
        {
        // 接收处理YAW轴信息
        case CAN_YAW_MOTOR_ID:
        {
            get_motor_measure(&motor_gimbal_data[0], rx_data);
            break;
        }

        default:
        {
            break;
        }
        }
    }
}

/*CAN1发送发射机构电机控制电流
( 左摩擦轮 , 右摩擦轮 , M3508转盘 , M2006丝杆)
*/
void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t push_turn, int16_t push_push)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = CAN_FRIC_ALL_ID;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = (fric_left >> 8);
    can1_send_data[1] = fric_left;
    can1_send_data[2] = ((-fric_right) >> 8);
    can1_send_data[3] = (-fric_right);
    can1_send_data[4] = (push_turn >> 8);
    can1_send_data[5] = push_turn;
    can1_send_data[6] = (push_push >> 8);
    can1_send_data[7] = push_push;
    // CAN1频道发送
    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}

// CAN2发送云台电机控制电流CAN2(0x205)
void CAN_cmd_gimbal(int16_t yaw)
{
    uint32_t send_mail_box;
    can2_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
    can2_send_data[0] = (yaw >> 8);
    can2_send_data[1] = yaw;
    can2_send_data[2] = 0;
    can2_send_data[3] = 0;
    can2_send_data[4] = 0;
    can2_send_data[5] = 0;
    can2_send_data[6] = 0;
    can2_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_send_data, &send_mail_box);
}

// 返回yaw 6020电机数据指针
const motor_measure_t *get_yaw_motor_measure_point(void)
{
    return &motor_gimbal_data[0];
}

// 返回摩擦轮左电机 3508电机数据指针
const motor_measure_t *get_left_fric_motor_measure_point(void)
{
    return &motor_fric_data[0];
}

// 返回摩擦轮右电机 3508电机数据指针
const motor_measure_t *get_right_fric_motor_measure_point(void)
{
    return &motor_fric_data[1];
}

// 返回推弹电机 3508电机数据指针
const motor_measure_t *get_push_3508_motor_measure_point(void)
{
    return &motor_push_data[0];
}

// 返回推弹电机 2006电机数据指针
const motor_measure_t *get_push_2006_motor_measure_point(void)
{
    return &motor_push_data[1];
}
