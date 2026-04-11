#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "main.h"
#include "math.h"
#include "bsp_can.h"
#include "Pid.h"
#include "motor_types.h"  // 包含基础类型定义
#include "Init.h"

//定义最大支持CAN口数量
#define MAX_CAN        3
// 定义最大支持的DJI电机数量
#define MAX_DJI_MOTORS 11
//角速度转化系数
#define rads   0.10472
#define MOTOR_ECD_TO_RAD 0.000766990394f

extern Motor_feedback DJI_Motor_RX[MAX_CAN][MAX_DJI_MOTORS];

uint16_t id_change(MotorType motor_type, uint8_t id);
uint8_t DJI_ID(uint8_t id);
void DJI_Motor_enable(MotorControlData *motors);
void DJI_Motor_disable(MotorControlData *motors);
void DJI3508_SpdClose_mode2(MotorInstance *motors);
void DJI6020_PosSpdClose_mode2(MotorInstance *motors);
void DJI2006_SpdClose_mode(MotorInstance *motors);
void update_total_angle(MotorControlData *m);
void DJI_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
void BM_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
#endif
