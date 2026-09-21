#ifndef __HERO_CHASSIS_H__
#define __HERO_CHASSIS_H__

#include "data_processing.h"
#include "Init.h"

#ifdef COMPILE_HERO_CHASSIS

#define Chassis_6020_offset_angle1   70.8f    //1号电机给的初始角度
#define Chassis_6020_offset_angle2   166.2f   //2号电机给的初始角度
#define Chassis_6020_offset_angle3    14.8f  //3号电机给的初始角度
#define Chassis_6020_offset_angle4   165.5f   //4号电机给的初始角度
#define PI      3.14159f///PI
#define drct    1
#define cos45   0.70711f
#define sin45   0.70711f
#define angle_change  57.29578f
#define Radius  45    //底盘中心到6020中心的半径

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

//底盘控制数据
typedef struct
{
    uint8_t gear_level;                //底盘速度挡位
    // 控制部分
    float vx;                           // 前进方向速度
    float vy;                           // 横移方向速度
    float w;                           // 旋转角速度
    float wheel3508_rpm[4];             // 3508速度
    float wheel6020_target_angle[4];    // 6020目标角度（绝对值角度）
    float wheel6020_relative_angle[4];  // 6020转的角度  
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;



void chassis_task(void *argument);
void Chassis_Init(MotorInstance *motors1, MotorInstance *motors2);
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors,  MotorInstance *motors1, uint8_t motor_count);
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void chassis_motor_updata(MotorInstance *motors1, MotorInstance *motors2);
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void  AGV_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void AGV_Motor_Set(MotorInstance  *motors1, MotorInstance  *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Chassis_PIDClear(MotorInstance  *motors1, MotorInstance  *motors2);
void Gear_Switch_Handler(void);
float Get_Current_Gear_Limit(void);
void KeyAccumulator_Dual_Instant_receive(uint16_t key_inc, uint16_t key_dec,
                               float *value, float step, float limit);

#endif

#endif
