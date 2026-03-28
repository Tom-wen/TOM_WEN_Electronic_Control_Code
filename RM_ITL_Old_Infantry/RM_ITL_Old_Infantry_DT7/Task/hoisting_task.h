#ifndef __HOISTING_TASK_H__
#define __HOISTING_TASK_H__

#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

#define PI 3.14159265358979323846f

#define HOISTING_TASK_INIT_TIME    357     // 任务开始空闲一段时间

typedef enum
{
  HOISTING_MOTOR_RAW = 0,   /**< 电机原始值未处理，云台电机未使能模式 */
  HOISTING_MOTOR_GYRO,
  AUTO_HOISTING,
  SET_UP_HOISTING     
} HOISTING_mode_e;

typedef struct
{
  pid_type_def hoisting_motor_relative_pid;
  pid_type_def hoisting_motor_gyro_pid;               /**< 角速度环PID */
  float max_relative_angle;                         /**< 最大相对角度，单位：弧度 */
  float min_relative_angle;                         /**< 最小相对角度，单位：弧度 */
  float relative_angle;                             /**< 相对角度，单位：弧度 */
  float relative_angle_set;                         /**< 相对角度设定值，单位：弧度 */
  float relative_angle_init;                        /**< 相对角度初始化值，单位：弧度 */
  float motor_gyro;                                 /**< 电机角速度，单位：弧度/秒 */
  float motor_gyro_set;                             /**< 电机角速度设定值，单位：弧度/秒 */
  float current_set;                                /**< 电流设定值 */
  int16_t given_current;                            /**< 给定电流值 */
} hoisting_motor_t;

typedef struct
{
  HOISTING_mode_e hoisting_mode;            /**< 当前控制模式 */
  hoisting_motor_t hoisting_motor[4];
} hoisting_t;

  // 堵转保护相关变量
static uint32_t stall_detect_time[4] = {0};  // 每个电机的堵转检测开始时间
static uint8_t stall_protection_flag[4] = {0};  // 堵转保护标志位

extern hoisting_t hoisting_control;

#define STALL_POSITION_THRESHOLD  (0.1f * 2.0f * PI)  // 0.1圈的弧度值
#define STALL_TIME_THRESHOLD      1000  // 1秒，单位ms

//motor 1
#define HOISTING_MOTOR_RELATIVE_KP_1 10
#define HOISTING_MOTOR_RELATIVE_KI_1 0
#define HOISTING_MOTOR_RELATIVE_KD_1 0
#define HOISTING_MOTOR_RELATIVE_MAX_OUT_1 100
#define HOISTING_MOTOR_RELATIVE_MAX_IOUT_1 100
#define HOISTING_MOTOR_GYRO_KP_1 20
#define HOISTING_MOTOR_GYRO_KI_1 0.1
#define HOISTING_MOTOR_GYRO_KD_1 0.0
#define HOISTING_MOTOR_GYRO_MAX_OUT_1 40.0
#define HOISTING_MOTOR_GYRO_MAX_IOUT_1 100

//motor 2
#define HOISTING_MOTOR_RELATIVE_KP_2 10
#define HOISTING_MOTOR_RELATIVE_KI_2 0
#define HOISTING_MOTOR_RELATIVE_KD_2 0
#define HOISTING_MOTOR_RELATIVE_MAX_OUT_2 100
#define HOISTING_MOTOR_RELATIVE_MAX_IOUT_2 100
#define HOISTING_MOTOR_GYRO_KP_2 20
#define HOISTING_MOTOR_GYRO_KI_2 0.1
#define HOISTING_MOTOR_GYRO_KD_2 0.0
#define HOISTING_MOTOR_GYRO_MAX_OUT_2 40.0
#define HOISTING_MOTOR_GYRO_MAX_IOUT_2 100

//motor 3
#define HOISTING_MOTOR_RELATIVE_KP_3 10
#define HOISTING_MOTOR_RELATIVE_KI_3 0
#define HOISTING_MOTOR_RELATIVE_KD_3 0
#define HOISTING_MOTOR_RELATIVE_MAX_OUT_3 100
#define HOISTING_MOTOR_RELATIVE_MAX_IOUT_3 100
#define HOISTING_MOTOR_GYRO_KP_3 20
#define HOISTING_MOTOR_GYRO_KI_3 0.1
#define HOISTING_MOTOR_GYRO_KD_3 0.0
#define HOISTING_MOTOR_GYRO_MAX_OUT_3 40.0
#define HOISTING_MOTOR_GYRO_MAX_IOUT_3 100

//motor 4
#define HOISTING_MOTOR_RELATIVE_KP_4 10
#define HOISTING_MOTOR_RELATIVE_KI_4 0
#define HOISTING_MOTOR_RELATIVE_KD_4 0
#define HOISTING_MOTOR_RELATIVE_MAX_OUT_4 100
#define HOISTING_MOTOR_RELATIVE_MAX_IOUT_4 100
#define HOISTING_MOTOR_GYRO_KP_4 20
#define HOISTING_MOTOR_GYRO_KI_4 0.1
#define HOISTING_MOTOR_GYRO_KD_4 0.0
#define HOISTING_MOTOR_GYRO_MAX_OUT_4 40.0
#define HOISTING_MOTOR_GYRO_MAX_IOUT_4 100


//初始角度前馈
#define forward_angle_feedforward 0.0f   //rad  
#define backward_angle_feedforward 0.0f  //rad


#define HOISTING_AUTO_FORWARD_ANGLE_OFFSET 3.0f//3.0
#define HOISTING_AUTO_BACKWARD_ANGLE_OFFSET 7.0f//7.0

//遥控器
#define FORWARD_CHANNEL 2
#define BACK_CHANNEL 3
#define RC_TO_HOISTING_DEADBAND 10

#define forward_RC_SEN 0.000015f
#define back_RC_SEN 0.000015f

#ifdef DT7_rc_ctrl
  #define HOISTING_MODE_CHANNEL LEFT_SWITCH
#endif

#ifdef i6x_rc_ctrl
  #define HOISTING_MODE_CHANNEL 3
#endif

/**
 * @brief 遥控器的死区判断
 * @param input 输入的遥控器值
 * @param output 输出的死区处理后遥控器值
 * @param dealine 死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

#endif

