/**
 * @file    gimbal_task.h
 * @brief   云台控制任务头文件
 * @details 定义云台控制相关的数据结构、宏定义和函数声明
 */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ******************************************************************
 * 宏定义 - 任务时间配置
 ****************************************************************** */
/** 云台任务初始化时间，单位：毫秒 */
#define GIMBAL_TASK_INIT_TIME 201

/** 云台控制周期 1ms */
#define GIMBAL_CONTROL_TIME 1

/* ******************************************************************
 * 宏定义 - 遥控器通道配置
 ****************************************************************** */
#ifdef DT7_rc_ctrl  
  /** yaw轴控制通道 */
  #define YAW_CHANNEL 2

  /** pitch轴控制通道 */
  #define PITCH_CHANNEL 3

  /** 云台模式控制通道 */
  #define GIMBAL_MODE_CHANNEL LEFT_SWITCH
#endif

#ifdef i6x_rc_ctrl  
  /** yaw轴控制通道 */
  #define YAW_CHANNEL 3

  /** pitch轴控制通道 */
  #define PITCH_CHANNEL 2

  /** 云台模式控制通道 */
  #define GIMBAL_MODE_CHANNEL 2
#endif
/* ******************************************************************
 * 宏定义 - 云台控制参数
 ****************************************************************** */
/** 云台手动调试速度 */
#define TURN_SPEED 0.04f

/** 遥控器摇杆死区 */
#define RC_DEADBAND 10

/** 判断云台静止的摇杆范围 */
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10

/** 云台静止最大时间，单位：毫秒 */
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

/* ******************************************************************
 * 宏定义 - 控制灵敏度参数
 ****************************************************************** */
/** Yaw轴遥控器灵敏度 */
#define YAW_RC_SEN -0.000012f

/** Pitch轴遥控器灵敏度 */
#define PITCH_RC_SEN 0.000012f

/** Yaw轴编码器灵敏度 */
#define YAW_ENCODE_SEN 0.01f

/** Pitch轴编码器灵敏度 */
#define PITCH_ENCODE_SEN 0.01f

//按键逃跑
#define KEY_RUN 1.54f

/* ******************************************************************
 * 宏定义 - 电机方向配置
 * 根据电机的安装方向取反电流
 ****************************************************************** */
#define PITCH_TURN 1
#define PITCH_OUT_TURN 0
#define YAW_TURN 1
#define YAW_OUT_TURN 1

/* ******************************************************************
 * 宏定义 - Pitch轴PID参数
 ****************************************************************** */
/* Pitch角度环控制参数 */
#define PITCH_ABSOLUTE_ANGLE_KP  5.0f //15.7f
#define PITCH_ABSOLUTE_ANGLE_KI  0.00001f //0.00001f
#define PITCH_ABSOLUTE_ANGLE_KD  0.67f //0.67f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT  30.0f //30.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT 1000.0f //1000.0f

/* Pitch角速度环控制参数 */
#define PITCH_GYRO_KP   7000.0f //10000.0f
#define PITCH_GYRO_KI   18.0f //15.0f
#define PITCH_GYRO_KD   0.2f //0.2f
#define PITCH_GYRO_MAX_OUT   16000.0f //16000.0f
#define PITCH_GYRO_MAX_IOUT 30000.0f //30000.0f

/* Pitch前馈电流参数 */
#define PITCH_CURRENT_FEEDFORWARD -7000.0f

/* ******************************************************************
 * 宏定义 - Yaw轴PID参数
 ****************************************************************** */
/* Yaw角度环控制参数 */
#define YAW_ABSOLUTE_ANGLE_KP   -10.0f //-30.0f
#define YAW_ABSOLUTE_ANGLE_KI   -0.01f //-0.01f
#define YAW_ABSOLUTE_ANGLE_KD   -0.8f //-0.8f
#define YAW_ABSOLUTE_ANGLE_MAX_OUT  5.0f //5.0f
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 2.0f //2.0f

/* Yaw角速度环控制参数 */
#define YAW_GYRO_KP   12000.0f //21000.0f
#define YAW_GYRO_KI   0.01f //0.0f
#define YAW_GYRO_KD   2.0f//1.8f
#define YAW_GYRO_MAX_OUT   16000.0f//16000.0f
#define YAW_GYRO_MAX_IOUT 1000.0f //1000.0f

/* ******************************************************************
 * 宏定义 - 云台小陀螺模式PID参数
 ****************************************************************** */
/* 小陀螺模式Yaw角度环控制参数 */
#define YAW_TOP_ABSOLUTE_ANGLE_KP   -10.0f
#define YAW_TOP_ABSOLUTE_ANGLE_KI   -0.01f
#define YAW_TOP_ABSOLUTE_ANGLE_KD   -0.8f
#define YAW_TOP_ABSOLUTE_ANGLE_MAX_OUT  5.0f
#define YAW_TOP_ABSOLUTE_ANGLE_MAX_IOUT 2.0f

/* 小陀螺模式Yaw角速度环控制参数 */
#define YAW_TOP_GYRO_KP   12000.0f
#define YAW_TOP_GYRO_KI   0.01f
#define YAW_TOP_GYRO_KD   2.0f
#define YAW_TOP_GYRO_MAX_OUT   16000.0f
#define YAW_TOP_GYRO_MAX_IOUT 1000.0f

/* ******************************************************************
 * 宏定义 - 云台机械限位角度值设定
 ****************************************************************** */
/** Yaw轴编码器偏置值 */
#define GIMBAL_YAW_OFFSET_ECD       847.8f

/** Yaw轴小陀螺模式编码器偏置值 */
#define GIMBAL_YAW_TOP_OFFSET_ECD   1400.0f

/** Yaw轴最大角度值，单位：弧度 */
#define GIMBAL_YAW_MAX_ECD          5.0f

/** Yaw轴最小角度值，单位：弧度 */
#define GIMBAL_YAW_MIN_ECD          -5.0f

/** Pitch轴编码器偏置值 */
#define GIMBAL_PITCH_OFFSET_ECD     0x1806

/** Pitch轴最大角度值，单位：弧度 */
#define GIMBAL_PITCH_MAX_ECD        0.544f

/** Pitch轴最小角度值，单位：弧度 */
#define GIMBAL_PITCH_MIN_ECD        -0.333f

/* ******************************************************************
 * 宏定义 - 编码器转换
 ****************************************************************** */
/** 电机编码器转换为角度值的转换系数 2*PI/8192 */
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f
#endif

/* ******************************************************************
 * 枚举定义 - 云台电机控制模式
 ****************************************************************** */
/**
 * @brief 云台电机控制模式枚举
 */
typedef enum
{
  GIMBAL_MOTOR_RAW = 0,   /**< 电机原始值未处理，云台电机未使能模式 */
  GIMBAL_MOTOR_GYRO,      /**< 使用陀螺仪绝对角度控制 */
  GIMBAL_MOTOR_ENCONDE    /**< 使用电机编码器相对角度控制 */
} gimbal_motor_mode_e;

/* ******************************************************************
 * 结构体定义 - 云台PID控制器
 ****************************************************************** */
/**
 * @brief 云台PID控制器结构体
 */
typedef struct
{
  float kp;           /**< 比例系数 */
  float ki;           /**< 积分系数 */
  float kd;           /**< 微分系数 */

  float set;          /**< 设定值 */
  float get;          /**< 反馈值 */
  float err;          /**< 误差 */

  float max_out;      /**< 最大输出 */
  float max_iout;     /**< 最大积分输出 */

  float Pout;         /**< 比例项输出 */
  float Iout;         /**< 积分项输出 */
  float Dout;         /**< 微分项输出 */

  float out;          /**< 总输出 */
} gimbal_PID_t;

/* ******************************************************************
 * 结构体定义 - 云台电机
 ****************************************************************** */
/**
 * @brief 云台电机结构体
 */
typedef struct
{
  const motor_measure_t *gimbal_motor_measure;      /**< 电机测量数据指针 */
  gimbal_PID_t gimbal_motor_absolute_angle_pid;     /**< 绝对角度环PID */
  gimbal_PID_t gimbal_motor_relative_angle_pid;     /**< 相对角度环PID */
  pid_type_def gimbal_motor_gyro_pid;               /**< 角速度环PID */
  gimbal_motor_mode_e gimbal_motor_mode;            /**< 当前控制模式 */
  gimbal_motor_mode_e last_gimbal_motor_mode;       /**< 上次控制模式 */
  float offset_ecd;                                 /**< 编码器偏置值 */
  float max_relative_angle;                         /**< 最大相对角度，单位：弧度 */
  float min_relative_angle;                         /**< 最小相对角度，单位：弧度 */

  float relative_angle;                             /**< 相对角度，单位：弧度 */
  float relative_angle_set;                         /**< 相对角度设定值，单位：弧度 */
  float absolute_angle;                             /**< 绝对角度，单位：弧度 */
  float absolute_angle_set;                         /**< 绝对角度设定值，单位：弧度 */
  float motor_gyro;                                 /**< 电机角速度，单位：弧度/秒 */
  float motor_gyro_set;                             /**< 电机角速度设定值，单位：弧度/秒 */
  float motor_speed;                                /**< 电机速度 */
  float raw_cmd_current;                            /**< 原始电流指令 */
  float current_set;                                /**< 电流设定值 */
  int16_t given_current;                            /**< 给定电流值 */
} gimbal_motor_t;

/* ******************************************************************
 * 结构体定义 - 云台控制
 ****************************************************************** */
/**
 * @brief 云台控制结构体
 */
typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;   /**< Yaw轴电机 */
  gimbal_motor_t gimbal_pitch_motor; /**< Pitch轴电机 */
} gimbal_control_t;

/* ******************************************************************
 * 全局变量声明
 ****************************************************************** */
extern gimbal_control_t gimbal_control;

/* ******************************************************************
 * 函数声明
 ****************************************************************** */
/**
 * @brief 获取Yaw电机结构体指针
 * @param[in] none
 * @retval Yaw电机结构体指针
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);


#endif