///* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

///* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ******************************************************************
                        AGV_CHASIS
****************************************************************** */
// 任务初始化时间 单位：毫秒
#define GIMBAL_TASK_INIT_TIME 201
// 云台控制周期 1ms
#define GIMBAL_CONTROL_TIME 1

// yaw, pitch通过遥控器通道状态通过通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH

// 云台手动调试
#define TURN_SPEED 0.04f
// 测试键盘按键未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
// 遥控器摇杆死区，摇杆在中间，摇杆范围，原始值为一个为0
#define RC_DEADBAND 10

// 控制灵敏度
#define YAW_RC_SEN   -0.000004f//-0.000005f // 0.005
#define PITCH_RC_SEN  0.000006f // 0.005

#define YAW_MOUSE_SEN 0.00009f  //0.00009f  //0.00005f
#define PITCH_MOUSE_SEN 0.000055f  //0.00003f//0.00010f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

// 判断遥控器摇杆是否在一定范围内，限制遥控器摇杆输入，使yaw的设定值不会因为摇杆漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

/////////////////////////// 根据电机的安装方向取反电流 ////////////////////////////////
#define PITCH_TURN 0
#define PITCH_OUT_TURN 0
#define YAW_TURN 1
#define YAW_OUT_TURN 1


///////////////////////////// 云台pid参数///////////////////////////////////
// pitch角度环控制函数角度，单位：弧度
#define PITCH_ABSOLUTE_ANGLE_KP  10.7f//10.7f     //7.7
#define PITCH_ABSOLUTE_ANGLE_KI 0.00001f
#define PITCH_ABSOLUTE_ANGLE_KD 0.67f//0.0001f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 20.0f    //20.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT  1000.0f // 积分项限制，debug
// pitch角速度环
#define PITCH_GYRO_KP 6500.0f//12000.0f//12000.0f      5500//6500       
#define PITCH_GYRO_KI 90.0f //10.0f
#define PITCH_GYRO_KD 10.2f              //10.8f
#define PITCH_GYRO_MAX_OUT 30000.0f//15000.0f
#define PITCH_GYRO_MAX_IOUT 30000.0f  //1000.0f

// yaw角度环控制函数角度，单位：弧度
#define YAW_ABSOLUTE_ANGLE_KP   -10.0f//10.12f //7.98f  //5.0f //10.18f//14.0f//4.2f   //5.6f     //  8.39f     //8.39f    //5.6f  10.11f//-10.0
#define YAW_ABSOLUTE_ANGLE_KI   0.0f   //0.0001f
#define YAW_ABSOLUTE_ANGLE_KD   0.5f//-0.95f  //-1.0f   //-0.95f   //-1.18f  //0.01f  0.2
#define YAW_ABSOLUTE_ANGLE_MAX_OUT 5.0f  //5.0f   //8.0f //15.0f   //9.0f   5.0f
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 1.0f  //3.0f
// yaw角速度环
#define YAW_GYRO_KP 21000.0f   //21000.0f  
#define YAW_GYRO_KI 0.0f  //0.0f
#define YAW_GYRO_KD 0.0f
#define YAW_GYRO_MAX_OUT 16000.0f
#define YAW_GYRO_MAX_IOUT 1000.0f

///////////////////////// 云台机械限位角度值设定 ////////////////////////////
#define GIMBAL_YAW_OFFSET_ECD           849.9f//6550//6851//4741//6666//6666//5.112758 电调反馈 6630//6666//0x1AFF-0x0400 //0.7762f    0x095c(-30)  0x19CF    0x1AFF-0x0400=19FF(-5)
#define GIMBAL_YAW_TOP_OFFSET_ECD           6851
#define GIMBAL_YAW_MAX_ECD             5.0f  // 最大值
#define GIMBAL_YAW_MIN_ECD             -5.0f // 最小值
#define GIMBAL_PITCH_OFFSET_ECD        0x1806   //6150
#define GIMBAL_PITCH_MAX_ECD           0.360f//0.360
#define GIMBAL_PITCH_MIN_ECD           -0.40f//-0.75f//0.40						

// 电机编码器转换为角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
  GIMBAL_MOTOR_RAW = 0, // 电机原始值未处理      云台电机未使能模式
  GIMBAL_MOTOR_GYRO,    // 使用陀螺仪绝对角度控制
  GIMBAL_MOTOR_ENCONDE  // 使用电机编码器相对角度控制
} gimbal_motor_mode_e;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float set;
  float get;
  float err;

  float max_out;
  float max_iout;

  float Pout;
  float Iout;
  float Dout;

  float out;
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  float offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad     // 编码器角度
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          获取yaw电机结构体
 * @param[in]      none
 * @retval         yaw电机结构体
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          获取pitch电机结构体
 * @param[in]      none
 * @retval         pitch电机结构体
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief 云台状态反馈给上位机
 *
 */
//extern void gimbal_data_send(void);

extern void gimbal_task(void const *pvParameters);

#endif