#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
/* =========================== 任务时间定义 =========================== */

#define CHASSIS_TASK_INIT_TIME    357     // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS   2       // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME      0.002f  // 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_FREQUENCE 500.0f  // 底盘任务控制频率

/* =========================== 遥控器通道定义 =========================== */
#ifdef DT7_rc_ctrl 
    /** 前后方向遥控器通道号 */
    #define CHASSIS_X_CHANNEL 1

    /** 左右方向遥控器通道号 */
    #define CHASSIS_Y_CHANNEL 0

    /** 特殊模式下控制旋转的遥控器通道号 */
    #define CHASSIS_WZ_CHANNEL 2

    /** 底盘模式选择开关通道号 */
    #define CHASSIS_MODE_CHANNEL LEFT_SWITCH
#endif

#ifdef i6x_rc_ctrl 
    /** 前后方向遥控器通道号 */
    #define CHASSIS_X_CHANNEL 1

    /** 左右方向遥控器通道号 */
    #define CHASSIS_Y_CHANNEL 0

    /** 特殊模式下控制旋转的遥控器通道号 */
    #define CHASSIS_WZ_CHANNEL 2

    /** 底盘模式选择开关通道号 */
    #define CHASSIS_MODE_CHANNEL 2
#endif

#ifdef vtm_rc_ctrl

   #define KEY_POSITIVE_X_SPEED 150.0f
   #define KEY_NEGATIVE_X_SPEED -150.0f
   #define KEY_POSITIVE_Y_SPEED 150.0f
   #define KEY_NEGATIVE_Y_SPEED -150.0f
   #define KEY_MAX_X_SPEED 400.0f
   #define KEY_MAX_Y_SPEED 400.0f


#endif
/* =========================== 底盘控制参数 =========================== */
/**
 * @defgroup Chassis_Control_Parameters 底盘控制参数
 * @brief 定义底盘运动控制的转换系数和阈值
 * @{
 */

/** 遥控器前进摇杆(max 660)到车体前进速度(m/s)的转换比例 */
#define CHASSIS_VX_RC_SEN 0.006f

/** 遥控器左右摇杆(max 660)到车体左右速度(m/s)的转换比例 */
#define CHASSIS_VY_RC_SEN 0.005f

/** 跟随底盘yaw模式下，遥控器yaw摇杆(max 660)到车体角度的增量比例 */
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

/** 不跟随云台时，遥控器yaw摇杆(max 660)到车体旋转速度的转换比例 */
#define CHASSIS_WZ_RC_SEN 0.01f

/** 遥控器摇杆死区值 */
#define CHASSIS_RC_DEADLINE 10

/** chassis_open模式下，遥控器值的缩放比例 */
#define CHASSIS_OPEN_RC_SCALE 10

/* =========================== 底盘电机速度PID参数 =========================== */

#define CHASSIS_MOTOR_SPEED_KP        9000
#define CHASSIS_MOTOR_SPEED_KI        0.0125
#define CHASSIS_MOTOR_SPEED_KD        0  
#define CHASSIS_MOTOR_SPEED_MAX_IOUT  1000

/* =========================== 底盘跟随云台旋转PID参数 =========================== */

#define CHASSIS_ANGLE_KP        16.0f   
#define CHASSIS_ANGLE_KI        0
#define CHASSIS_ANGLE_KD        0.001f
#define CHASSIS_ANGLE_MAX_OUT   8.5f    
#define CHASSIS_ANGLE_MAX_IOUT  1.5f

/* =========================== 底盘速度转换系数 =========================== */

#define CHASSIS_ACCEL_X_NUM                0.1666666667f
#define CHASSIS_ACCEL_Y_NUM                0.3333333333f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX    0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY    0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ    0.25f
#define MOTOR_DISTANCE_TO_CENTER           0.2f
#define M3508_MOTOR_RPM_TO_VECTOR          0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    M3508_MOTOR_RPM_TO_VECTOR

/* =========================== 底盘速度限制 =========================== */

#define MAX_WHEEL_SPEED           3.7f    // 单个底盘电机最大速度
#define CHASSIS_MOTOR_SPEED_MAX_OUT 20000.0f   // 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 6.0f 
#define NORMAL_MAX_CHASSIS_SPEED_Y 6.0f    // 底盘运动过程最大平移速度 
#define NORMAL_MAX_CHASSIS_SPEED_Z 6.0f    // 
#define CHASSIS_TOP_SPEED          10.0f       // 底盘小陀螺速度 
#define CHASSIS_FOLLOW_SPEED       10.0f        // 底盘旋转云台速度
#define TOP_MAX_CHASSIS_SPEED      6.0f    // 底盘小陀螺时最大平移速度

/* =========================== 底盘模式枚举 =========================== */

typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   // 底盘无力, 跟没上电那样
  CHASSIS_NO_MOVE,          // 底盘保持不动
  CHASSIS_FOLLOW_GIMBAL,    // 底盘跟随云台旋转
  CHASSIS_TOP,              // 底盘小陀螺模式
  CHASSIS_NO_FOLLOW_GIMBAL, // 底盘不跟随云台旋转
  CHASSIS_OPEN              // 遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_mode_e;

/* =========================== 底盘电机结构体 =========================== */

typedef struct
{
  const motor_measure_t *chassis_motor_measure;  // 电机测量数据指针
  float accel;           // 加速度
  float speed;           // 当前速度
  float speed_set;       // 设定速度
  int16_t give_current;  // 发送电流
} chassis_motor_t;

/* =========================== 底盘运动控制结构体 =========================== */

typedef struct
{
  const gimbal_motor_t *chassis_yaw_motor;   // 底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  const gimbal_motor_t *chassis_pitch_motor; // 底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const float *chassis_INS_angle;            // 获取陀螺仪解算出的欧拉角指针
  
  chassis_mode_e chassis_mode;               // 底盘控制状态机
  chassis_motor_t motor_chassis[4];          // 底盘电机数据
  pid_type_def motor_speed_pid[4];           // 底盘电机速度pid
  pid_type_def chassis_angle_pid;            // 底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx; // 使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy; // 使用一阶低通滤波减缓设定值

  float vx;                         // 底盘速度 前进方向 前为正，单位 m/s
  float vy;                         // 底盘速度 左右方向 左为正 单位 m/s
  float wz;                         // 底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                     // 底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                     // 底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                     // 底盘设定旋转角速度，逆时针为正 单位 rad/s
  
  float chassis_relative_angle;     // 底盘与云台的相对角度，单位 rad
  float chassis_relative_angle_set; // 设置相对云台控制角度
  float chassis_yaw_set;

  float vx_positive_max_speed;  // 前进方向最大速度 单位m/s
  float vx_negative_max_speed;  // 后退方向最大速度 单位m/s
  float vy_positive_max_speed;  // 左方向最大速度 单位m/s
  float vy_negative_max_speed;  // 右方向最大速度 单位m/s
  float top_positive_max_speed; // 小陀螺平移正方向最大速度
  float top_negative_max_speed; // 小陀螺平移反方向最大速度
  
  float chassis_yaw;   // 陀螺仪和云台电机叠加的yaw角度
  float chassis_pitch; // 陀螺仪和云台电机叠加的pitch角度
  float chassis_roll;  // 陀螺仪和云台电机叠加的roll角度
} chassis_move_t;




/* =========================== 函数声明 =========================== */

/**
 * @brief 底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param pvParameters: 空
 * @retval none
 */
extern void chassis_task(void const *pvParameters);



// 获取云台电机指针
const gimbal_motor_t* get_pitch_motor_point(void);
const gimbal_motor_t* get_yaw_motor_point(void);

extern chassis_move_t chassis_move;


#endif
