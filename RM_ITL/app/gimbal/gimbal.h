/**
 * @file    gimbal.h
 * @brief   云台控制任务头文件
 * @details 包含云台控制相关的数据结构、宏定义和函数声明
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

/* =========================== 包含头文件 =========================== */
#include "main.h"
#include "motor_types.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "ins.h"
#include "Init.h"
#include "user_lib.h"

#ifdef COMPILE_GIMBAL

/* =========================== 云台控制灵敏度 =========================== */

/** Yaw轴灵敏度 (遥控器值 -> 角度增量) */
#define YAW_SENSITIVITY        0.0004f
/** Pitch轴灵敏度 (遥控器值 -> 角度增量) */
#define PITCH_SENSITIVITY      0.0005f

/** 弧度转角度系数 */
#define HUDU 57.29578f

/** 遥控器死区值 */
#define GIMBAL_RC_DEADLINE 10

/* =========================== 云台机械限位 =========================== */

/** 云台YAW轴编码器回零偏移 */
#define GIMBAL_YAW_OFFSET      122.3f
/** 云台PITCH轴回零偏移 */
#define GIMBAL_PITCH_OFFSET    -5.80f
/** PITCH轴向上最大角度（度） */
#define PITCH_UP_MAX           -10.0f
/** PITCH轴向下最大角度（度） */
#define PITCH_DOWN_MAX          18.8f



/* =========================== YAW轴手动模式串级PID参数 =========================== */

#define YAW_MANUAL_OUTER_KP      8.0f
#define YAW_MANUAL_OUTER_KI      0.0f
#define YAW_MANUAL_OUTER_KD      0.0f
#define YAW_MANUAL_OUTER_MAX_I   10.0f
#define YAW_MANUAL_OUTER_MAX_OUT 190.0f
#define YAW_MANUAL_OUTER_KFF     20.0f

#define YAW_MANUAL_INNER_KP      180.0f
#define YAW_MANUAL_INNER_KI      0.0f
#define YAW_MANUAL_INNER_KD      0.0f
#define YAW_MANUAL_INNER_MAX_I   10.0f
#define YAW_MANUAL_INNER_MAX_OUT 10000.0f
#define YAW_MANUAL_INNER_KFF     0.0f

/* =========================== YAW轴自瞄模式串级PID参数 =========================== */

#define YAW_AUTO_OUTER_KP        14.5f
#define YAW_AUTO_OUTER_KI        0.0f
#define YAW_AUTO_OUTER_KD        0.0f
#define YAW_AUTO_OUTER_MAX_I     10.0f
#define YAW_AUTO_OUTER_MAX_OUT   160.0f
#define YAW_AUTO_OUTER_KFF       100.0f

#define YAW_AUTO_INNER_KP        215.0f
#define YAW_AUTO_INNER_KI        0.0f
#define YAW_AUTO_INNER_KD        0.0f
#define YAW_AUTO_INNER_MAX_I     10.0f
#define YAW_AUTO_INNER_MAX_OUT   11500.0f
#define YAW_AUTO_INNER_KFF       250.0f

/* =========================== PITCH轴串级PID参数 =========================== */

#define PITCH_OUTER_KP           38.0f
#define PITCH_OUTER_KI           0.0f
#define PITCH_OUTER_KD           0.0f
#define PITCH_OUTER_MAX_I        0.0f
#define PITCH_OUTER_MAX_OUT      165.0f
#define PITCH_OUTER_KFF          0.0f

#define PITCH_INNER_KP           45.0f
#define PITCH_INNER_KI           0.0f
#define PITCH_INNER_KD           0.0f
#define PITCH_INNER_MAX_I        300.0f
#define PITCH_INNER_MAX_OUT      16000.0f
#define PITCH_INNER_KFF          0.0f

/* =========================== 云台模式枚举 =========================== */

/**
 * @brief 云台控制模式
 */
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 电流零输入模式
    GIMBAL_NO_FOLLOW,         // 底盘控制云台不动模式
    GIMBAL_NORMAL,            // 正常遥控器控制模式
    GIMBAL_ABSOLUTE_ANGLE,    // 绝对角度模式（小陀螺时使用）
} gimbal_mode_e;

/* =========================== 自瞄状态枚举 =========================== */

/**
 * @brief 自瞄开关状态
 */
typedef enum
{
    AUTO_AIM_OFF = 0,         // 自瞄关闭
    AUTO_AIM_ON = 1,          // 自瞄开启
} auto_aim_mode;

/* =========================== 云台控制数据结构体 =========================== */

/**
 * @brief 云台控制命令结构体
 */
typedef struct
{
    float yaw;                  // YAW轴控制量
    float yaw_vel;              // YAW轴角速度
    float yaw_acc;              // YAW轴角加速度
    float pitch;                // PITCH轴控制量
    float pitch_vel;            // PITCH轴角速度
    float pitch_acc;            // PITCH轴角加速度
    float chassis_rotate_wz;    // 底盘旋转角速度
    float bullet_speed;         // 弹速
    uint16_t bullet_count;      // 子弹累计发送次数
    gimbal_mode_e gimbal_mode;  // 云台控制模式
} Gimbal_Ctrl_Cmd_s;

/**
 * @brief 云台测量数据结构体（用于视觉系统）
 */
typedef struct
{
    float yaw;       // YAW角度（度）
    float yaw_vel;   // YAW角速度 rad/s
    float pitch;     // PITCH角度（度）
    float pitch_vel; // PITCH角速度 rad/s
} Gimbal_measure;

/* =========================== 外部变量声明 =========================== */

extern Gimbal_Ctrl_Cmd_s gimbal_cmd_control;
extern float pitch_angle;
extern uint8_t auto_aim_flag;

/* =========================== 函数声明 =========================== */

/**
 * @brief 云台主任务函数
 * @param argument 任务参数
 */
void gimbal_task(void *argument);

/**
 * @brief 云台初始化函数
 * @param motors 电机实例数组指针
 */
void Gimbal_Init(MotorInstance *motors);

/**
 * @brief 清除云台PID控制器积分项
 * @param motors 电机实例数组指针
 */
void Gimbal_PIDClear(MotorInstance *motors);

/**
 * @brief 设置云台电机状态（使能/失能）
 * @param status 电机状态
 * @param motors 电机实例数组指针
 * @param motor_count 电机数量
 */
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);

/**
 * @brief 更新云台电机控制（发送指令）
 * @param motors 电机实例数组指针
 */
void gimbal_motor_updata(MotorInstance *motors);

/**
 * @brief pitch轴重力补偿
 * @param pitch 当前pitch角度（度）
 * @return 补偿电流值
 */
float Gravity_compensation(float pitch);

/**
 * @brief 编码器值转换为PITCH角度
 * @param motors PITCH轴电机实例指针
 * @return PITCH角度值（度）
 */
float Get_Pitch_Angle_From_Encoder(MotorInstance *motors);

#endif // COMPILE_GIMBAL
#endif // __GIMBAL_H__
