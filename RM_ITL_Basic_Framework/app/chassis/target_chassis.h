#ifndef __SENTRY_CHASSIS_H__
#define __SENTRY_CHASSIS_H__

#include "Init.h"


#ifdef TARGET_CHASSIS

#define sensitivity 0.003f
#define w_z 3.0f
//底盘模块
// 定义物理参数
#define WHEEL_RADIUS  0.10f   // 车轮半径 (米)
#define CHASSIS_GEAR_RATIO    10.0f   // 减速比，例如 10:1,相当于电机转十圈轮子转一圈
// 计算转换系数：m/s → 电机转子 RPM
#define MPS_TO_MOTOR_RPM  ((60.0f * CHASSIS_GEAR_RATIO) / (2.0f * 3.1415926f * WHEEL_RADIUS))

//云台模块
#define REV_TO_RADS  (2.0f * 3.1415926535f)
#define GIMBAL_GEAR_RATIO 2.0f/3.0f // 云台减速比

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 旋转靶模式
    CHASSIS_FOLLOW_GIMBAL, // 平移模式
} chassis_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    float v;                           // 前进方向速度 m/s
    float wheel3508_rpm[2];             // 3508速度rpm
    chassis_mode_e chassis_mode;
    chassis_mode_e chassis_last_mode;
} Chassis_Ctrl_Cmd_s;

//云台控制数据
typedef struct
{
    // 控制部分
    float w;                           // 靶车旋转速度 rad/s
    float fdb_w;                        // 靶车反馈速度
    float target_current;             // 5047电机目标电流
}Gimbal_Ctrl_Cmd_s;

void Target_Chassis_PID_Init(MotorInstance *motors1, MotorInstance *motors2);
void chassis_motor_update(MotorInstance *motors1, MotorInstance *motors2);
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd , Gimbal_Ctrl_Cmd_s *Gimbal_Cmd,MotorInstance *Chassis_3508,MotorInstance *Gimbal_5047) ;
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Target_control(MotorInstance *motor_chassis,Chassis_Ctrl_Cmd_s *Chassis_Cmd,MotorInstance *motors_gimbal,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);
void Motor_Status(Motor_status status, MotorInstance *motors,  MotorInstance *motors1, uint8_t motor1_count ,uint8_t motor2_count);
void Motor_Set(MotorInstance *motor_chassis,Chassis_Ctrl_Cmd_s *Chassis_Cmd,MotorInstance *motors_gimbal,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);

#endif

#endif
