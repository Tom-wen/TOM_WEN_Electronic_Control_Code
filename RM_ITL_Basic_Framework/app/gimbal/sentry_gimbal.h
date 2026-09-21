#ifndef __SENTRY_GIMBAL_H__
#define __SENTRY_GIMBAL_H__

#include "Init.h"
#include "ins.h"

#ifdef AUTO_SENTRY


#define hudu 57.29578f//弧度转角度
#define RPM_TO_RADS  (3.14159265358979323846f / 30.0f)

//大小yaw位置调下面的参数
#define pitch_up_max       -10.0f       //pitch向上最大值，编码器值2348
#define pitch_down_max       18.8f      //pitch向下最大值，编码器值3369
#define small_yaw_target_ecd -0.1978f //单位为度 范围-180到180
#define big_yaw_chassis_ecd   67.08f  //单位为度 范围-180到180 这个是指当大yaw和底盘都处于正方向时的编码器值
#define ecd_error_dead_zone 0.0f    //编码器误差死区，单位为度

//yaw轴灵敏度
#define yaw_sensitivity        0.0016f
//pitch轴灵敏度
#define pitch_sensitivity        0.0004f

#define SCAN_SPEED          30.0f       // 云台扫描速度（度/秒），正值顺时针

#define Gravity_compensation -0.4f      // 重力补偿
#define w_out_comp          1.0f     // 小陀螺情况下对大yaw输出的补偿

//云台控制模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 电流零输入模式
    GIMBAL_NO_FOLLOW,         // 底盘控制云台不动模式
    GIMBAL_NORMAL,            // 遥控器控制模式(云台跟随底盘)  
    GIMBAL_ABSOLUTE_ANGLE,    // 云台绝对值角度模式(小陀螺)
    GIMBAL_AUTO,               //导航自瞄hang 模式
} gimbal_mode_e;
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
    LqrBasic lqr;                     // yaw和pitch LQR设置
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;





void gimbal_motor_updata(MotorInstance *motors1, MotorInstance *motors2, MotorInstance *motors3);
void Gimbal_Init(MotorInstance *motors_1, MotorInstance *motors_2, MotorInstance *motors_3);
void RemoteControlGimbal(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);
void Gimbal_mode_update(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd);
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors_1, MotorInstance *motors_2,MotorInstance *motors_3 ,uint8_t motor_count);
void Gimbal_control(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw, MotorInstance *motors_pitch,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, proto_link_t *vision_feedback, INS_t *ins);
void control_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, INS_t *ins);
void Auto_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, proto_link_t *vision_feedback, INS_t *ins);
void limited_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd , float small_yaw_ecd_target , float big_yaw_ecd_target,float pitch_ecd_target);
void Scan_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw, MotorInstance *motors_pitch, INS_t *ins);
void Motor4310_detect_enable(void);
void LQR_Controller_Loop(MotorInstance* Motor_Small_Yaw ,float small_yaw_target,float small_yaw_vel_target,MotorInstance* Motor_Pitch ,float pitch_target,float pitch_vel_target , Gimbal_Ctrl_Cmd_s* gimbal_ctrl);
void Big_Yaw_Chassis_Relative_Angle_Update(MotorInstance* Motor_Big_Yaw);

#endif
#endif