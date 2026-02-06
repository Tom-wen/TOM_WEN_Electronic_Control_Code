#ifndef __GQ_MOTOR_H_
#define __GQ_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"
#include "PID.h"
#include "string.h"
#include "can.h"
#include "stdio.h"
#include "math.h"
#include <stdbool.h>  




typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;
typedef struct
{
  PID_struct_t gimbal_motor_relative_angle_pid;
  float max_relative_angle; // rad
  float min_relative_angle; // rad
  float relative_angle;     // rad
  float relative_angle_set; // rad     // 编码器角度
  float current_set;
  float given_current;

  uint8_t motor_id;

  float radar_add;  //雷达发送的增量角度

} gimbal_motor_t;

extern gimbal_motor_t gimbal_motor_t_yaw;
extern gimbal_motor_t gimbal_motor_t_pitch;

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF,CAN_HandleTypeDef* hcan);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode,CAN_HandleTypeDef* hcan);
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd,CAN_HandleTypeDef* hcan);
void Emm_V5_Restore_Motor(uint8_t addr,CAN_HandleTypeDef* hcan);







#ifdef __cplusplus
}
#endif

#endif
