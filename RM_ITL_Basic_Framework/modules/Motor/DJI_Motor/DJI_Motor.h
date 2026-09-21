#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "main.h"
#include "math.h"
#include "bsp_can.h"
#include "Pid.h"
#include "motor_types.h"  // 包含基础类型定义

// 定义最大支持的DJI电机数量
#define MAX_DJI_MOTORS 11
//角速度转化系数
#define rads   0.10472

extern Motor_feedback DJI_Motor_RX[MAX_CAN][MAX_DJI_MOTORS];

uint16_t id_change(MotorType motor_type, uint8_t id);
uint8_t DJI_ID(uint8_t id);
void DJI_Motor_enable(MotorControlData *motors);
void DJI_Motor_disable(MotorControlData *motors);
void DJI3508_Spd_mode(MotorInstance *motors);
void DJI3508_SpdClose_mode(MotorInstance *motors);
void DJI3508_AngleSpdClose_mode2(MotorInstance *motors);
void DJI3508_SpdClose_mode2(MotorInstance *motors);
 #ifdef COMPILE_M_POWER
  void DJI3508_SpdClose_mode2_MasterPower(MotorInstance *motors);
  #endif
void DJI3508_PosSpdClose_mode2(MotorInstance *motors);
void DJI6020_Voltage_mode(MotorInstance *motors);
void DJI6020_SpdClose_mode(MotorInstance *motors);
void DJI6020_SpdClose_mode2(MotorInstance *motors);
void DJI6020_PosClose_mode(MotorInstance *motors);
void DJI6020_PosClose_MultiTurn(MotorInstance *motors);
void DJI6020_PosSpdClose_mode2(MotorInstance *motors);
void DJI6020_current_mode(MotorInstance *motors);
void DJI2006_current_mode(MotorInstance *motors);
void DJI2006_SpdClose_mode(MotorInstance *motors);
void DJI2006_PosSpdClose_mode2(MotorInstance *motors);
void update_total_angle(MotorControlData *m);
void DJI_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
void BM_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
#endif
