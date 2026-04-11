#ifndef __MECANUM_CHASSIS_H__
#define __MECANUM_CHASSIS_H__

#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "math.h"
#include "Init.h"
#include "user_lib.h"
#include "chassis_behaviour.h"


//云台回零角度
#define yaw_offset      122.3f  //YAW
#define yaw_encoder_offset      2790//YAW编码器中值(小陀螺使用)
#define CHASSIS_WZ_SET_SCALE 0.1f
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define PI      3.1415926
#define LENGTH_A 0.4
#define LENGTH_B 0.6



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
   #define KEY_NEGATIVE_Y_SPEED -150.0f//150
   #define KEY_MAX_X_SPEED 300.0f
   #define KEY_MAX_Y_SPEED 300.0f//400


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
#define CHASSIS_MOTOR_SPEED_MAX_OUT   20000.0f
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
#define M3508_R          0.072f//底盘轮子的半径
#define rads   0.10472f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    M3508_R*rads

/* =========================== 底盘速度限制 =========================== */

#define MAX_WHEEL_SPEED           3.7f    // 单个底盘电机最大速度
#define CHASSIS_MOTOR_SPEED_MAX_OUT 20000.0f   // 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 6.0f 
#define NORMAL_MAX_CHASSIS_SPEED_Y 6.0f    // 底盘运动过程最大平移速度 
#define NORMAL_MAX_CHASSIS_SPEED_Z 6.0f    // 
#define CHASSIS_TOP_SPEED          6.5f       // 底盘小陀螺速度 
#define CHASSIS_TOP_SPEED_GEAR_LOW   CHASSIS_TOP_SPEED//低速小陀螺
#define CHASSIS_TOP_SPEED_GEAR_HIGH  (1.5f * CHASSIS_TOP_SPEED)//高速小陀螺
#define CHASSIS_FOLLOW_SPEED       10.0f        // 底盘旋转云台速度
#define TOP_MAX_CHASSIS_SPEED      6.0f    // 底盘小陀螺时最大平移速度





typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_NO_FOLLOW_GIMBAL,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL, // 跟随模式，底盘叠加角度环控制
    CHASSIS_TOP,            // 小陀螺模式
} chassis_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    float vx_set;                          // 前进方向速度
    float vy_set;                          // 横移方向速度m/s
    float w_set;                           // 旋转角速度rad/s

    float vx;                              // 前进方向实际速度
    float vy;                              // 横移方向实际速度
    float w;                              // 旋转实际速度
    
    chassis_mode_e chassis_mode;

    first_order_filter_type_t chassis_cmd_slow_set_vx; // 使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy; // 使用一阶低通滤波减缓设定值

    float chassis_relative_angle;     // 底盘与云台的相对角度，单位 rad
    float chassis_relative_angle_set; // 设置相对云台控制角度
    float chassis_yaw_set;
    pid_type_def chassis_angle_pid;            // 底盘跟随角度pid


} Chassis_Ctrl_Cmd_s;

extern Chassis_Ctrl_Cmd_s chassis_cmd_move;

void chassis_task(void *argument);
void Chassis_Init(MotorInstance *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd);
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void chassis_motor_updata(MotorInstance *motors);

#endif

