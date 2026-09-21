#ifndef __UAV_GIMBAL_H__
#define __UAV_GIMBAL_H__

#include "ins.h"
#include "Init.h"

#ifdef COMPILE_UAV_GIMBAL

//yaw轴灵敏度
#define yaw_sensitivity        0.0001f//0.0004f
//pitch轴灵敏度
#define pitch_sensitivity        0.00025f//0.0005f

//传入实际电流值（0.10能正常转动，尽量不要超过0.2）
#define current_change  5461.333f
//转速转为角速度系数（rpm/min-->rad/s）
#define rad_change  0.104720f
//弧度转角度
#define hudu 57.29578
//云台回零角度
#define yaw_offset          176.0f    //YAW
#define pitch_offset        -5.80f    //PITCH
#define pitch_max           0.2f    //pitch最大值（向上）
#define pitch_min           37.2f     //pitch最小值（向下）
#define pitch_encoder_max   2315.0f   //pitch编码器最上
#define pitch_encoder_min   3157.0f   //pitch编码器最下
#define yaw_encoder_left    2900
#define yaw_encoder_right   5290
//云台控制模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 电流零输入模式
    GIMBAL_NORMAL,            // 遥控器控制模式(云台跟随底盘)  
    GIMBAL_ABSOLUTE_ANGLE,    // 云台绝对值角度模式(小陀螺)
} gimbal_mode_e;

//自瞄状态
typedef enum
{
    AUTO_AIM_ON = 1,             // 自瞄开启
    AUTO_AIM_OFF = 0,            // 自瞄关闭
} auto_aim_mode;

//云台控制数据(接收上位机数据)
typedef struct
{
    // 控制部分
    float yaw;              // 调式使用
    float yaw_vel;          // 调式使用
    float yaw_acc;
    float pitch;            // 调式使用
    float pitch_vel;        // 调式使用
    float pitch_acc;
    float chassis_rotate_wz;
    float bullet_speed;     // 弹速
    uint16_t bullet_count;  // 子弹累计发送次数
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

//测量的云台参数
typedef struct 
{
    float yaw;       // 角度弧度
    float yaw_vel;   // 角速度 rad/s
    float pitch;
    float pitch_vel;
} Gimbal_measure;

extern float pitch_angle;
extern float yaw_angle;

void gimbal_task(void *argument);
void Gimbal_Init(MotorInstance *motors);
void Gimbal_PIDClear(MotorInstance  *motors);
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void Gimbal_mode_update(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);
void RemoteControlGimbal(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);
void gimbal_motor_updata(MotorInstance *motors);
void Gimbal_control(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, VisionToGimbal *vision_feedback, INS_t *ins);
void remote_control(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, INS_t *ins);
void Auto_aiming_cal(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, VisionToGimbal *vision_feedback, INS_t *ins);
void Gimbal_Measure(Gimbal_measure *measure_data, INS_t *ins, MotorInstance *motors);
void Gimbal_PID_Change(MotorInstance *motors);
float Gravity_compensation(float pitch);
float Get_Pitch_Angle_From_Encoder(MotorInstance *motor_pitch);
float Get_Yaw_Angle_From_Encoder(MotorInstance *motors);
#endif

#endif
