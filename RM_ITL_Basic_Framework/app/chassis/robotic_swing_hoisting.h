#ifndef __ROBOTIC_SWING_HOISTING_H__
#define __ROBOTIC_SWING_HOISTING_H__

#include "Init.h"
#include <math.h>

#ifdef ROBOTIC_SWING_CHASSIS
#include "robotic_swing_chassis.h"
#include "Pid.h"

typedef enum
{
  HOISTING_MOTOR_OFF = 0,    // 电机关闭
  SET_OFF_HOISTING,          // 放下抬升
  SET_UP_HOISTING,           // 抬升
  AUTO_LEVEL_ADJUST,          // 自动调平
  SET_UP_SWING_HOISTING       //摆臂交替
} hoisting_mode_e;


//底盘控制数据
typedef struct
{
    float forward_hoisting;
    float backward_hoisting;
    float motor_init_angle[4]; //单位rad
    float motor_target_angle[4];
    float motor_target_current[4];
    float motor_normalized_pos[4];       // 归一化反馈位置 [-0.5, 0.5] 单位：圈
    int32_t motor_revolution_count[4];   // 累计整圈数
    hoisting_mode_e hoisting_mode;
    PID   leveling_pitch_pid;            // 俯仰调平PID
    PID   leveling_roll_pid;             // 横滚调平PID
    float pitch_adjustment;              // 俯仰调节量 rad
    float roll_adjustment;               // 横滚调节量 rad
} Hoisting_Ctrl_Cmd_s;

#define HOISTING_AUTO_FORWARD_ANGLE_OFFSET 9.0f//摆臂前臂抬升 单位rad//3.0,9.0
#define HOISTING_AUTO_BACKWARD_ANGLE_OFFSET 9.0f//

#define HOISTING_AUTO_RIGHT_ANGLE_OFFSET 7.0f
#define HOISTING_AUTO_LEFT_ANGLE_OFFSET 4.0f
#define switchtime 20

#define LEVELING_PID_KP       0.8f    // 调平P增益
#define LEVELING_PID_KI       0.0f    // 调平I增益
#define LEVELING_PID_KD       0.0f    // 调平D增益
#define LEVELING_PID_MAX_I    5.0f    // 调平积分限幅
#define LEVELING_PID_MAX_OUT  10.8f    // 调平输出限幅 current
#define LEVELING_DEAD_ZONE_DEG 30.0f   // 调平死区 度

void Hoisting_Init(MotorInstance *motors, Hoisting_Ctrl_Cmd_s *Hoisting_Cmd);
void Hoisting_mode_update(Hoisting_Ctrl_Cmd_s *Hoisting_Cmd);
void Hoisting_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void Hoisting_control(MotorInstance *Hoisting_5047, Hoisting_Ctrl_Cmd_s *Hoisting_Cmd);
void Hoisting_motor_updata(MotorInstance *motors);

#endif

#endif