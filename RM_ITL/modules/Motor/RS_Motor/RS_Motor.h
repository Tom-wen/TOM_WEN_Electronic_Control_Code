#ifndef __RS_MOTOR_H__
#define __RS_MOTOR_H__

#include "main.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "motor_types.h"  // 包含基础类型定义

//定义最大支持CAN口数量
#define MAX_CAN        3
// 定义最大支持的RS电机数量
#define MAX_RS_MOTORS 11
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f 
#define V_MAX 44.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

typedef enum
{
    rs_mit_mode = 0,    //MIT模式
    rs_pos_mode = 1,    //位置模式
    rs_vel_mode = 2     //速度模式
}RS_mode;

extern Motor_feedback RS_Motor_RX[MAX_CAN][MAX_RS_MOTORS];

uint16_t RS_ID_Change(uint16_t id);
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits);
void RS_Motor_Init(MotorInstance *motors, uint8_t mode);
void RS_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t id);
void RS_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t id);
void RS_Motor_ID_Change(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint16_t target_id);
void RS_Motor_Set_mode(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t mode);
void RS_Motor_Speed_mode(MotorInstance *motors);
void RS_Motor_Pos_mode(MotorInstance *motors);
void RS_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);
#endif
