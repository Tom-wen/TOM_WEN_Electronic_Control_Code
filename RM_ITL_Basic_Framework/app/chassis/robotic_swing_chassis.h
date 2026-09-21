#ifndef __ROBOTIC_SWING_CHASSIS_H__
#define __ROBOTIC_SWING_CHASSIS_H__

#include "Init.h"
#include "Master_power.h"
#ifdef ROBOTIC_SWING_CHASSIS

//云台回零角度
#define yaw_offset      0.3f  //YAW
#define yaw_encoder_offset     0//YAW编码器中值(小陀螺使用)
#define CHASSIS_WZ_SET 14.0f //rad/s

#define sensitivity        0.01f//控制的灵敏度


#define WHEEL_RADIUS        0.25f   // 轮子的半径（单位m）
#define REDUCTION_RATIO     19.0f    // 减速比 19:1
#define PI      3.14159265358979323846f
#define RADPS_TO_RPM        (60.0f / (2.0f * PI))   // 1 rad/s = 9.5493 rpm
#define MPS_TO_WHEEL_RPM    (RADPS_TO_RPM / WHEEL_RADIUS)  // 1 m/s 对应的轮子 rpm
#define MPS_TO_MOTOR_RPM    (MPS_TO_WHEEL_RPM * REDUCTION_RATIO)  // 1 m/s 对应的电机 rpm
#define MOTOR_DISTANCE_TO_CENTER 0.7f//LENGTH_A + LENGTH_B  半轴距 + 半轮距

extern uint8_t chassis_online;

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_DEAD,               // 写死模式
} chassis_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    float vx;                          // 前进方向速度m/s
    float vy;                          // 横移方向速度
    float w;                           // 旋转角速度rad/s
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
void Check_Chassis_Offline(void);
#endif

#endif
