///**
// * @file gimbal_task.c/h
// * @author 陈冠华, 林晓锋
// * @brief 云台控制任务
// * @version 0.1
// * @date 2022-03-06
// *
// * @copyright Copyright (c) 2022 SPR
// *
// */
// 
// /*
//***********************某琳不弯腰，宁愿抬较之头，不愿抬之腰，谢谢团队***********************
//*
//                        1 1 1 1 1 1 1 1 1 1                 1
//                                          1                 1
//                                          1                 1
//                                          1                 1
//                                          1                 1
//                                          1                 1
//                                          1                 1
//                                          1                 1
//                        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 
//                        1                 1
//                        1                 1
//                        1                 1
//                        1                 1
//                        1                 1
//                        1                 1
//                        1                 1
//                        1                 1 1 1 1 1 1 1 1 1 1 
//												
//***********************某琳不弯腰，宁愿抬较之头，不愿抬之腰，谢谢团队***********************

//*/
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "main.h"
#include "INS_task.h"
#include "arm_math.h"
#include "math.h"
#include "detect_task.h"
#include "user_lib.h"
//#include "USART_receive.h"
//#include "config.h"
#include "chassis_task.h"
#include "usart.h"
//#define debug_mode 1
extern chassis_move_t chassis_move;
// PID参数定义
#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

/**
 * @brief          初始化"gimbal_control"结构体变量，pid初始化，遥控器输入设定值设定，惯导模块获取设定值设定
 * @retval         none
 */
static void gimbal_init(void);

/**
 * @brief          设置云台电机模式，需要调用'gimbal_behaviour_mode_set'函数修改
 * @retval         none
 */
static void gimbal_set_mode(void);

/**
 * @brief          反馈数据更新，包括编码器，速度，角度，电机转速
 * @retval         none
 */
static void gimbal_feedback_update(void);

/**
 * @brief          根据ecd和offset_ecd计算角度变化
 * @param[in]      ecd: 电机编码器数值
 * @param[in]      offset_ecd: 电机中位编码器数值
 * @retval         编码器角度，单位rad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          设置云台电机设定值，通过gimbal_behaviour_control_set函数获取
 * @retval         none
 */
static void gimbal_set_control(void);

/**
 * @brief          执行循环反馈控制，根据设定值计算输出值
 * @retval         none
 */
static void gimbal_control_loop(void);

/**
 * @brief 云台状态反馈给上位机
 *
 */
//void gimbal_data_send(void);

/**
 * @brief          用于GIMBAL_MOTOR_GYRO模式的角度设定，防止电机卡死
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          用于GIMBAL_MOTOR_ENCONDE模式的角度设定，防止电机卡死
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          云台电机模式：GIMBAL_MOTOR_GYRO 使用陀螺仪和绝对角度进行控制
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台电机模式：GIMBAL_MOTOR_ENCONDE 使用编码器相对角度进行控制
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief 云台PID初始化函数
 *
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param maxout 最大输出
 * @param max_iout 最大积分输出
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout);

/**
 * @brief 云台PID计算函数
 *
 * @param pid PID结构体指针
 * @param get 当前值
 * @param set 设定值
 * @param error_delta 误差变化量，用于微分项
 * @return fp32 控制输出
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta);

/**
 * @brief          清除云台pid结构体的out, iout
 * @param[out]     pid_clear: "gimbal_control"结构体指针
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

// 云台控制结构体
gimbal_control_t gimbal_control;

// 底盘移动结构体
extern chassis_move_t chassis_move;

// 调试数组
float gimbal_debug[10] = {0};
float gimbal_debug1[10] = {0};

// CAN发送电流值
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;

/**
 * @brief 云台控制任务主函数
 * @param pvParameters RTOS任务参数
 */
void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  // 云台初始化
//  gimbal_init();

//  // 判断是否上电成功
//  while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) 
//  {
//    vTaskDelay(GIMBAL_CONTROL_TIME);
//    gimbal_feedback_update(); // 云台反馈更新
//  }

  while (1)
  {
    // 设置云台电机模式
    gimbal_set_mode();
    
    // 云台反馈更新
    gimbal_feedback_update();
    
    // 设置云台电机设定值
    gimbal_set_control();
    
    // 云台电机PID控制
    gimbal_control_loop();
    
    // 云台状态反馈给上位机（供gimbal_behaviour使用）
//    gimbal_data_send();

    // 根据电机的安装方向，调整电流方向
    #if YAW_TURN
      yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
    #else
      yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
    #endif
    
    #if PITCH_TURN
      pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
    #else
      pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
    #endif

    // 检查电机是否正常
    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
    {
      if (toe_is_error(DBUS_TOE)) // 遥控器离线
      {
        CAN_cmd_gimbal(0, 0);
        // 停止失控保护
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
      }
      else
      {
        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current);
      }
    }

    // 调试模式输出
    #if debug_mode
      if (toe_is_error(YAW_GIMBAL_MOTOR_TOE))
      {
        gimbal_debug[7] = 5;
      }
      else if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
      {
        gimbal_debug[7] = -5;
      }
      
      gimbal_debug[0] = gimbal_control.gimbal_pitch_motor.absolute_angle;
      gimbal_debug[1] = gimbal_control.gimbal_pitch_motor.absolute_angle_set;
      gimbal_debug[2] = gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.out;
      gimbal_debug[3] = gimbal_control.gimbal_pitch_motor.given_current;
      gimbal_debug[4] = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->given_current;
      gimbal_debug[5] = auto_shoot.pitch_add;
      gimbal_debug[6] = auto_shoot.yaw_add;
      
      gimbal_debug1[0] = gimbal_control.gimbal_yaw_motor.absolute_angle;
      gimbal_debug1[1] = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
    #endif

    vTaskDelay(GIMBAL_CONTROL_TIME);
  }
}

/**
 * @brief          初始化"gimbal_control"结构体变量
 * @retval         none
 */
static void gimbal_init(void)
{
  // 电机结构体获取
  gimbal_control.gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();     // 获取yaw电机编码器角度
  gimbal_control.gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point(); // 获取pitch电机编码器角度
  
  // 初始化电机模式
  gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = gimbal_control.gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = gimbal_control.gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  
  // 初始化云台变量和限位
  gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_ECD;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_ECD;

  gimbal_control.gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ECD;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_ECD;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_ECD;

  // 初始化yaw电机pid
  gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
                  YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, 
                  YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT);
  PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, 
           YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, 
           YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
  
  // 初始化pitch电机pid
  gimbal_PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, 
                  PITCH_ABSOLUTE_ANGLE_KP, PITCH_ABSOLUTE_ANGLE_KI, PITCH_ABSOLUTE_ANGLE_KD, 
                  PITCH_ABSOLUTE_ANGLE_MAX_OUT, PITCH_ABSOLUTE_ANGLE_MAX_IOUT);
  PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, 
           PITCH_GYRO_KP, PITCH_GYRO_KI, PITCH_GYRO_KD, 
           PITCH_GYRO_MAX_OUT, PITCH_GYRO_MAX_IOUT);

  // 清除所有PID数据
  gimbal_total_pid_clear(&gimbal_control);
  
  // 反馈更新
  gimbal_feedback_update();
  
  // 初始化电机状态
  gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
  gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
  gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

  gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
  gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_pitch_motor.relative_angle;
  gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

//  // 初始化电机发送指针
//  user_send_data.gimbal_pitch_angle = &gimbal_control.gimbal_pitch_motor.absolute_angle;
//  user_send_data.gimbal_yaw_gyro = &gimbal_control.gimbal_yaw_motor.motor_gyro;
}

/**
 * @brief          设置云台电机模式
 * @retval         none
 */
static void gimbal_set_mode()
{
  gimbal_behaviour_mode_set(&gimbal_control);
}

/**
 * @brief          反馈数据更新
 * @retval         none
 */
static void gimbal_feedback_update()
{
  // 根据电机安装方向，计算编码器角度
  #if PITCH_TURN
    gimbal_control.gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(
      gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, 
      gimbal_control.gimbal_pitch_motor.offset_ecd);
  #else
    gimbal_control.gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(
      gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, 
      gimbal_control.gimbal_pitch_motor.offset_ecd);
  #endif
  
  gimbal_control.gimbal_pitch_motor.absolute_angle = INS_data.angle_pitch;
  gimbal_control.gimbal_pitch_motor.motor_gyro = INS_data.wy;

  #if YAW_TURN
    gimbal_control.gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(
      gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, 
      gimbal_control.gimbal_yaw_motor.offset_ecd);
  #else
    gimbal_control.gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(
      gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, 
      gimbal_control.gimbal_yaw_motor.offset_ecd);
  #endif
  
  gimbal_control.gimbal_yaw_motor.absolute_angle = INS_data.angle_yaw;
  // 计算yaw轴角速度，考虑pitch角的影响
  gimbal_control.gimbal_yaw_motor.motor_gyro = -(cosf(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wz 
                                                 - sinf(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wx);
}

/**
 * @brief          根据ecd和offset_ecd计算角度变化
 * @param[in]      ecd: 电机编码器数值
 * @param[in]      offset_ecd: 电机中位编码器数值
 * @retval         编码器角度，单位rad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  
  // 处理编码器过零
  if (relative_ecd > 4096)
  {
    relative_ecd -= 8191;
  }
  else if (relative_ecd < -4096)
  {
    relative_ecd += 8191; 
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          设置云台电机设定值
 * @retval         none
 */
extern gimbal_behaviour_e gimbal_behaviour;

static void gimbal_set_control(void)
{
    float add_yaw_angle = 0.0f;
    float add_pitch_angle = 0.0f;


   //gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);
    
//    // 使用自动射击模块的目标值
//    gimbal_control.gimbal_yaw_motor.absolute_angle_set = auto_shoot.yaw_add;
//    gimbal_control.gimbal_pitch_motor.absolute_angle_set = auto_shoot.pitch_add;
    
    // // // 注释掉的前一版本逻辑
  
     if (gimbal_behaviour == GIMBAL_AUTO)//检测状态
     {
         gimbal_control.gimbal_yaw_motor.absolute_angle_set = add_yaw_angle;
         gimbal_control.gimbal_pitch_motor.absolute_angle_set = add_pitch_angle;
     }
     else
     {
         gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);
         gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + add_yaw_angle;
         gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, add_pitch_angle);
     }

}

/**
 * @brief          用于GIMBAL_MOTOR_GYRO模式的角度设定，防止电机卡死
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    static float bias_angle;
    static float angle_set;
    
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    // 当前角度偏差
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    
    // 检查是否会超出机械限位
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        if (add > 0.0f)
        { 
            // 限制添加的角度
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          用于GIMBAL_MOTOR_ENCONDE模式的角度设定，防止电机卡死
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    gimbal_motor->relative_angle_set += add;
    
    // 角度限幅
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

/**
 * @brief          执行循环反馈控制
 * @retval         none
 */
static void gimbal_control_loop()
{
    // 根据底盘模式选择不同的PID参数
    if (chassis_move.chassis_mode == CHASSIS_TOP)   // 小陀螺模式
    {
        gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
                        YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, -0.497f, 
                        YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT);
        PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, 
                 YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, 
                 YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
    }
    else  // 正常模式
    {
        gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
                        YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, 
                        YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT);
        PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, 
                 YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, 
                 YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
    }
    
    // yaw轴控制
    if (gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_control.gimbal_yaw_motor.given_current = 0;
    }
    else
    { 
        gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_yaw_motor);
    }

    // pitch轴控制
    if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_control.gimbal_pitch_motor.given_current = 0;
    }
    else
    {
        gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_pitch_motor);
    }
}

/**
 * @brief          云台电机绝对角度控制
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    // 角度环计算目标角速度
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
                                                   gimbal_motor->absolute_angle, 
                                                   gimbal_motor->absolute_angle_set, 
                                                   gimbal_motor->motor_gyro);
    
    // 速度环计算目标电流
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, 
                                         gimbal_motor->motor_gyro, 
                                         gimbal_motor->motor_gyro_set);
    
    // 设定值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台电机相对角度控制
 * @param[out]     gimbal_motor: yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    // 角度环计算目标角速度
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
                                                   gimbal_motor->relative_angle, 
                                                   gimbal_motor->relative_angle_set, 
                                                   gimbal_motor->motor_gyro);
    
    // 速度环计算目标电流
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, 
                                         gimbal_motor->motor_gyro, 
                                         gimbal_motor->motor_gyro_set);
    
    // 设定值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set); 
}

/**
 * @brief 云台PID初始化函数
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param maxout 最大输出
 * @param max_iout 最大积分输出
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

/**
 * @brief 云台PID计算函数
 * @param pid PID结构体指针
 * @param get 当前值
 * @param set 设定值
 * @param error_delta 误差变化量，用于微分项
 * @return fp32 控制输出
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta)
{
    float err;
    
    if (pid == NULL)
    {
        return 0.0f;
    }
    
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    
    // PID计算
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    
    // 限幅
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    
    return pid->out;
}

/**
 * @brief          清除云台pid结构体的数据
 * @param[out]     gimbal_pid_clear: PID结构体指针
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
 * @brief          获取yaw电机结构体指针
 * @retval         yaw电机结构体指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          获取pitch电机结构体指针
 * @retval         pitch电机结构体指针
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

///**
// * @brief 云台状态反馈给上位机
// */
//void gimbal_data_send(void)
//{
//    static uint8_t send_time;
//    
//    // 每5个循环发送一次（500Hz控制频率）
//    if (send_time == 2)
//    {
//        user_data_pack_handle();
//        send_time = 0;
//    }
//    
//    send_time++;
//}