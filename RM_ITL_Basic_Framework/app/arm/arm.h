#ifndef __ARM_H__
#define __ARM_H__

#include "Init.h"

#ifdef COMPILE_ARM
//夹爪遥控器控制灵敏度
#define claw_sensitivity    0.002
#define ARM6036_MAX_POS     4800
#define ARM6036_MIN_POS     -4400
#define ARM4310_J1_MAX_POS  1.9
#define ARM4310_J1_MIN_POS  -1.5
#define ARM4310_J2_MAX_POS  2.2
#define ARM4310_J2_MIN_POS  -0.89
#define ARM4310_J2_START_POS  0.72      //初始位置
#define CLAW_LEFT_START_POS    4.668    //左边是电机2，往上角度(1)
#define CLAW_RIGHT_START_POS   1.411    //右边是电机1，往上角度(0)
#define CLAW_LEFT_UPMAX_POS    6.300    //向上最大
#define CLAW_RIGHT_UPMAX_POS   -0.231   //向上最大
#define CLAW_LEFT_DOWNMAX_POS    3.127  //向下最大
#define CLAW_RIGHT_DOWNMAX_POS   2.958  //向下最大
#define CLAW_LEFT_LEFTMAX_POS    3.142    //向左最大
#define CLAW_RIGHT_LEFTMAX_POS   -0.229   //向左最大
#define CLAW_LEFT_RIGHTMAX_POS    6.314  //向右最大
#define CLAW_RIGHT_RIGHTMAX_POS   2.942  //向右最大
typedef enum
{
    ARM_ZERO_FORCE = 0,                 // 电流零输入
    ARM_CONTROL_MODE = 1,               // 机械臂键盘模式
    ARM_REMOTE_MODE = 2,

} arm_mode_e;

typedef struct 
{
    float arm_speed;          //机械臂电机速度
    float arm_position;       //机械臂电机位置
    float arm_current;        //机械臂电机电流大小
} Arm_Motor_Data;

typedef struct 
{
    float claw_speed;          //夹爪电机速度
    float claw_position;       //夹爪电机位置
    float claw_current;        //夹爪电机电流大小
} Claw_Motor_Data;

//底盘控制数据
typedef struct
{   
    // 电机控制数据
    Arm_Motor_Data Arm6036_Motor[1];    //机械臂高擎电机
    Arm_Motor_Data Arm4310_Motor[2];    //机械臂达妙电机
    Claw_Motor_Data Claw_Motor[2];      //夹爪电机
    // 控制部分
    arm_mode_e arm_mode;
} Arm_Ctrl_Cmd_s;

void arm_task(void *argument);
void Arm_Init(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3);
void Arm_Motor_Status(Motor_status status, MotorInstance *motors, MotorInstance *motors1, MotorInstance *motors2);
void RemoteControlArm(Arm_Ctrl_Cmd_s *Arm_Cmd);
void Arm_mode_update(Arm_Ctrl_Cmd_s *Arm_Cmd);
void arm_motor_updata(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3);
void Arm_control(Arm_Ctrl_Cmd_s *Arm_Cmd);
void Arm_Motor_Set(Arm_Ctrl_Cmd_s *Arm_Cmd, MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3);
void User_define_control(float *angle, uint16_t *motor_target_angle);
#endif 

#endif
