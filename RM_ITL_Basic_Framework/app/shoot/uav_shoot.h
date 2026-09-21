#ifndef __UAV_SHOOT_H__
#define __UAV_SHOOT_H__

#include "Init.h"

#ifdef COMPILE_UAV_SHOOT

typedef enum
{
    SHOOT_ZERO_FORCE = 0,    // 电流零输入
    SHOOT_NORMAL = 1,        // 摩擦轮正常工作模式
    FIRE_MODE = 2            // 发射模式   
} shoot_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    float left_speed;                  // 左摩擦轮电机速度
    float right_speed;                 // 右摩擦轮电机速度
    float trigger_speed;               // 拨弹电机速度
    shoot_mode_e shoot_mode;
} Shoot_Ctrl_Cmd_s;

void shoot_task(void *argument);
void Shoot_Init(MotorInstance *motors, MotorInstance *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void Shoot_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void Shoot_mode_update(Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void shoot_motor_updata(MotorInstance *motors, MotorInstance *motors2);
void RemoteControlShoot(Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void Shoot_Motor_Set(MotorInstance  *motors, MotorInstance  *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd, VisionToGimbal *vision_feedback);
void Shoot_PIDClear(MotorInstance  *motors);

#endif

#endif
