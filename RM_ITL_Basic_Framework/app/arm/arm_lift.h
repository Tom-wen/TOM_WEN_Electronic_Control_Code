#ifndef __ARM_LIFT_H__
#define __ARM_LIFT_H__

#include "data_processing.h"
#include "Init.h"

#ifdef COMPILE_ARM_LIFT

typedef enum
{
    ARM_LIFT_ZERO_FORCE = 0,    // 电流零输入
    ARM_LIFT_NORMAL = 1,            // 小陀螺模式
    ARM_LIFT_REMOTE = 2,
} arm_lift_mode_e;

//底盘控制数据
typedef struct
{
    float lift_speed;
    float claw_speed;
    arm_lift_mode_e lift_mode;
} Arm_Lift_Ctrl_Cmd_s;

void arm_lift_task(void *argument);
void arm_lift_motor_updata(MotorInstance *motors1, MotorInstance *motors2);
void Arm_Lift_Init(MotorInstance *motors1, MotorInstance *motors2);
void Arm_Lift_Motor_Status(Motor_status status, MotorInstance *motors,  MotorInstance *motors1, uint8_t motor_count);
void Arm_Lift_mode_update(Arm_Lift_Ctrl_Cmd_s *lift_Cmd);
void Arm_Lift_control(Arm_Lift_Ctrl_Cmd_s *lift_Cmd);
void Arm_Lift_Motor_Set(MotorInstance  *motors1, MotorInstance  *motors2, Arm_Lift_Ctrl_Cmd_s *lift_Cmd);

#endif

#endif
