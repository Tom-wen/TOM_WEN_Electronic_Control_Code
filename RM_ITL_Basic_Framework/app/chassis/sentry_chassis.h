#ifndef __SENTRY_CHASSIS_H__
#define __SENTRY_CHASSIS_H__

#include "Init.h"


#ifdef AUTO_SENTRY

/* 运动学常量 */
#define PI      3.14159f
#define cos45   0.70711f
#define sin45   0.70711f
#define angle_change  57.29578f       // 弧度转角度系数（180/π）
#define Radius  0.4f                  // 底盘中心到6020中心的半径（m）
#define GEAR_RATIO 15.76f             // 减速比
#define WHEEL_RADIUS   0.0525f        // 轮子半径（m）
#define RPM_TO_RADS  (3.14159265358979323846f / 30.0f)  // RPM转rad/s
#define hudu 57.29578f                // 弧度转角度

/* 底盘参数 */
#define Chassis_6020_offset_angle1   258.45f   // 1号6020电机初始角度
#define Chassis_6020_offset_angle2   224.43f   // 2号6020电机初始角度
#define Chassis_6020_offset_angle3   12.81f    // 3号6020电机初始角度
#define Chassis_6020_offset_angle4   104.76f   // 4号6020电机初始角度
#define yaw_offset  245.1783f                  // 云台正向时电机编码器值（0-360°)大yaw编码器
#define sensitivity 0.0132f                    // 遥控器灵敏度
#define WZ_PID_DEADZONE  0.0f                // 云台跟随PID输出死区
#define WZ_ANGLE_DEADZONE  5.0f              // 云台跟随角度死区（度）

#define drct    1

extern const float feed_forward_rpm_speed[4];

/* 底盘模式枚举 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_AUTO,              // 导航模式
    CHASSIS_DEAD,              // 写死模式
} chassis_mode_e;

/* 底盘控制数据结构体 */
typedef struct
{
    float vx;                           // 前进方向速度（m/s）
    float vy;                           // 横移方向速度（m/s）
    float w;                            // 旋转角速度（rad/s）
    float vx_fdb;                       // 反馈的前进方向速度（m/s）
    float vy_fdb;                       // 反馈的横移方向速度（m/s）
    float w_fdb;                        // 反馈的旋转角速度（rad/s）
    PID wz_pid[3];                      // 底盘跟随云台旋转PID
    float wheel3508_rpm[4];             // 3508目标转速
    float wheel6020_target_angle[4];    // 6020目标角度（绝对值）
    float wheel6020_last_target_angle[4];
    float wheel6020_relative_angle[4];  // 6020相对转角
    float wheel6020_current_angle[4];   // 6020当前连续角度（累计编码器角度，度）
    chassis_mode_e chassis_mode;
    chassis_mode_e chassis_last_mode;
} Chassis_Ctrl_Cmd_s;

extern Chassis_Ctrl_Cmd_s chassis_cmd_send;//这里声明结构体，在云台解算时需要用到
/*
* @brief 死控运动阶段定义
*/
typedef struct
{
    int32_t duration_ticks;   // 阶段时长（tick，1 tick≈4ms）
    float vx;                 // 目标前进速度（m/s）
    float vy;                 // 目标横移速度（m/s）
    float w;                  // 目标角速度（rad/s）
} Dead_Phase_s;

/*
* @brief 死控运动阶段参数表
* @note  需要调参时直接在此处修改各阶段时长与速度即可
* 阶段时长（tick，1 tick≈4ms）目标前进速度（m/s）目标横移速度（m/s）目标角速度（rad/s）
*/
static const Dead_Phase_s dead_plan[] =
{
    {2000,  0.0f,   0.0f,   0.0f},
    {500,   0.5f,   0.0f,   0.0f},
    {500,   0.0f,  -0.25f,  0.0f},
    {500,  -0.5f,   0.0f,   0.0f},
    {500,   0.0f,   0.25f,  0.0f},
    {500,   0.0f,   0.0f,   0.0f},
};
#define DEAD_PLAN_CNT   (sizeof(dead_plan) / sizeof((dead_plan)[0]))

/* 任务与初始化 */
void chassis_task(void *argument);
void Chassis_Init(MotorInstance *motors1, MotorInstance *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd);

/* 电机控制 */
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors, MotorInstance *motors1, uint8_t motor_count);
void chassis_motor_updata(MotorInstance *motors1, MotorInstance *motors2);

/* 模式管理 */
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void chassis_dead_control(Chassis_Ctrl_Cmd_s *Chassis_Cmd, chassis_mode_e last_mode);
void Chassis_control(MotorInstance *Motor1, MotorInstance *Motor2, Chassis_Ctrl_Cmd_s *Chassis_Cmd);

/* 遥控器输入 */
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd);

/* AGV运动学 */
void AGV_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd, MotorInstance *Chassis_6020);
void AGV_Speed_Feedback(MotorInstance *motors1, MotorInstance *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void AGV_Motor_Set(MotorInstance *motors1, MotorInstance *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Absolute_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd, float angle);
float Chassis_relative_angle(MotorInstance *motors);

/* 云台跟随 */
void Set_follow_wz(MotorInstance *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd);

/* 辅助 */
void Chassis_PIDClear(MotorInstance *motors1, MotorInstance *motors2);

#endif

#endif
