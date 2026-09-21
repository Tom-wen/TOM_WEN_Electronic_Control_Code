#ifndef __M_HERO_CHASSIS_H__
#define __M_HERO_CHASSIS_H__

#include "data_processing.h"
#include "Init.h"

#ifdef COMPILE_M_HERO_CHASSIS

//云台回零角度
#define yaw_offset      19.996f  //YAW
#define yaw_encoder_offset      0.349f//YAW弧度中值(小陀螺使用)
#define CHASSIS_WZ_SET_SCALE 0.1f
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define PI      3.1415926
#define LENGTH_A 0.4
#define LENGTH_B 0.6
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_ROTATE,            // 小陀螺模式
} chassis_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    uint8_t gear_level;                //底盘速度挡位
    float vx;                          // 前进方向速度
    float vy;                          // 横移方向速度
    float w;                           // 旋转角速度
    float wheel3508_rpm[4];            // 3508速度
    PID cascade_pid[3];                // 底盘跟随云台旋转PID
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

extern Chassis_Ctrl_Cmd_s chassis_cmd_send;

void chassis_task(void *argument);
void Chassis_Init(MotorInstance *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void chassis_motor_updata(MotorInstance *motors);
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Chassis_PIDClear(MotorInstance  *motors);
void Absolute_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd, float angle);
void Mecanum_Calc(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Mecanum_Motor_Set(MotorInstance  *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Set_follow_wz(MotorInstance  *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
float Chassis_relative_angle(MotorInstance  *motors);
void Gear_Switch_Handler(void);
float Get_Current_Gear_Limit(void);
void power_data_sent(MotorInstance *motors, MotorPower* power_motor, uint8_t motor_count);

#endif
#endif