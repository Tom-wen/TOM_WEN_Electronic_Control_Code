#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__
#include "stm32h7xx.h"
#include "motor_types.h"  // 首先包含基础类型定义
#include "bsp_can.h"
#include "bsp_dwt.h"

//电机KP,KD范围
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

//定义最大支持CAN口数量
#define MAX_CAN        3
// 定义最大支持的DM电机数量
#define MAX_DM_MOTORS 8

//电机模式
typedef enum
{
	mit_mode = 1, //MIT模式
	pos_mode = 2, //位置速度模式
	spd_mode = 3, //速度模式
	psi_mode = 4 //力位混控模式控制
}DM_mode;

// 电机参数设置结构体
typedef struct
{
    float PMAX;		// 位置映射范围
    float VMAX;		// 速度映射范围
    float TMAX;		// 扭矩映射范围
}motor_t;

extern Motor_feedback DM_Motor_RX[MAX_CAN][MAX_DM_MOTORS];

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_Motor_Init(MotorInstance *motors);
void motor_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void DM_Motor_enable(MotorControlData *motors);
void DM_Motor_disable(MotorControlData *motors);
void DM_Mit_mode(MotorInstance *motors);
void DM_Pos_mode(MotorInstance *motors);
void DM_Spd_mode(MotorInstance *motors);
void DM_Psi_mode(MotorInstance *motors);
void read_motor_rx(FDCAN_HandleTypeDef *hfdcan, uint16_t id);
void read_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid);
void write_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid);
void write_save_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void Motor4310_enable_judge(MotorInstance *motors);
void DM_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port);

#endif
