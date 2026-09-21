#ifndef __ROBOTIC_SWING_SHOOT_H__
#define __ROBOTIC_SWING_SHOOT_H__

#include "Init.h"
#include "robotic_swing_gimbal.h"

#ifdef ROBOTIC_SWING_SHOOT


typedef enum
{
    SHOOT_ZERO_FORCE = 0,    // 电流零输入
    SHOOT_NORMAL = 1,        // 摩擦轮正常工作模式
    FIRE_MODE = 2 ,           // 发射模式 
    SHOOT_CLOSE = 3,         // 摩擦轮关闭模式  
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

/*======================= 发射延迟测试参数 =======================*/
#define FIRE_LATENCY_ARM_SPEED     6450.0f  // 摩擦轮转速达到该值(绝对值)认为已加速到位，开始等待弹丸
#define FIRE_LATENCY_DROP_SPEED    100.0f   // 摩擦轮相对额定转速的掉速量，超过则判定弹丸已被送出
#define FIRE_LATENCY_SETTLE_MS     50.0f    // 摩擦轮加速到位后的稳定时间，期间不判定掉速，避开超调

extern float fire_latency_us;               // 最近一次测量结果，单位us

void shoot_task(void *argument);
void Shoot_Init(MotorInstance *motors, MotorInstance *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void Shoot_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void Shoot_mode_update(Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void shoot_motor_updata(MotorInstance *motors, MotorInstance *motors2);
void RemoteControlShoot(Shoot_Ctrl_Cmd_s *Shoot_Cmd);
void Shoot_Motor_Set(MotorInstance  *motors, MotorInstance  *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd, VisionToGimbal *vision_feedback);
void Shoot_PIDClear(MotorInstance  *motors);
float Fire_Latency_Test(MotorInstance *motors, Shoot_Ctrl_Cmd_s *Shoot_Cmd);

#endif

#endif
