#ifndef __GQ_MOTOR_H__
#define __GQ_MOTOR_H__

#include "bsp_can.h"
#include "bsp_dwt.h"

//定义最大支持CAN口数量
#define MAX_CAN        3
// 定义最大支持的RS电机数量
#define MAX_GQ_MOTORS 11

extern Motor_feedback GQ_Motor_RX[MAX_CAN][MAX_GQ_MOTORS];

void GQ_Motor_Init(MotorInstance *motors);
void motor_control_vel(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t vel, int16_t tqe);
void GQ_Motor_Speed_mode(MotorInstance *motors);
void GQ_Motor_Tqe_mode(MotorInstance *motors);
void GQ_Motor_Pos_Vel_mode(MotorInstance *motors);
void timed_return_motor_status(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t t_ms);
void GQ_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
#endif
