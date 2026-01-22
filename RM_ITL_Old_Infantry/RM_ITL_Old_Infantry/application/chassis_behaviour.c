
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"

/* ==================== 宏定义 ==================== */

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

/* ==================== 全局变量声明 ==================== */

extern void chassis_power_control_update(void);

// 键盘模式控制标志
uint8_t keyboard_mode_up = 0;      // 键盘上挡模式（小陀螺）
uint8_t keyboard_mode_mid = 0;     // 键盘中挡模式（跟随云台）
uint8_t keyboard_mode_down = 0;    // 键盘下挡模式（不跟随）

// 键盘按键状态标志
static uint8_t top_mode_flag = 0;         // 小陀螺模式切换标志
// static int unfollow_flag = 0;       // 不跟随模式切换标志（已注释）

// 小陀螺速度控制变量
static uint8_t top_mode_speed = CHASSIS_TOP_SPEED;
static uint8_t top_speed_toggle_flag = 0; // 小陀螺速度切换标志

/* ==================== 静态函数声明 ==================== */

/**
 * @brief 底盘无力控制 - 底盘模式为raw，将设定值都设置为0
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 底盘不移动控制 - 底盘模式不跟随角度
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 底盘跟随云台控制 - 底盘跟随云台角度，根据角度差计算旋转角速度
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 底盘小陀螺控制 - 一边旋转一边以云台指向方向运动
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 底盘不跟随角度控制 - 底盘不跟随角度，旋转速度由参数直接设定
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 底盘开环控制 - 底盘为raw原生状态，设定值直接发送到CAN总线
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/* ==================== 函数实现 ==================== */

/**
 * @brief 根据遥控器通道值，计算纵向和横移速度
 * @param[out] vx_set 纵向速度指针
 * @param[out] vy_set 横向速度指针
 * @param[in] chassis_move_rc_to_vector 底盘数据指针
 * @retval none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  float vx_set_channel, vy_set_channel;

  // 死区限制，因为遥控器可能存在差异，摇杆在中间其值不为0
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  // 键盘控制 - 前后
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_W)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_S)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  // 键盘控制 - 左右
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_A)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_D)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  // 一阶低通滤波代替斜坡作为底盘速度输入（平滑）
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

  // 在死区内，停止信号，不需要缓慢加速，直接减速到零
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && 
      vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && 
      vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = vx_set_channel;
  *vy_set = vy_set_channel;
}

/**
 * @brief 底盘模式选择
 * @param[in] chassis_move_mode 底盘数据
 * @retval none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }

  // 遥控器设置模式（非必要不建议改）
  if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 上挡
  {
    chassis_move_mode->chassis_mode = CHASSIS_TOP;
  }
  else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 中挡
  {
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
  }
  else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 下挡
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
  }

  // 键盘数据清空 - Z键
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_Z)
  {
    keyboard_mode_up = 0;
    keyboard_mode_mid = 1;
    keyboard_mode_down = 0;
  }

  // 根据键盘标志设置模式
  if (keyboard_mode_up == 1)
  {
    chassis_move_mode->chassis_mode = CHASSIS_TOP;
  }
  if (keyboard_mode_mid == 1)
  {
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
  }
  if (keyboard_mode_down == 1)
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
  }

  /* ==================== 键盘控制 ==================== */

  // 小陀螺模式切换 - F键
  if (!(rc_ctrl.key & KEY_PRESSED_OFFSET_F))
  {
    top_mode_flag = 1;
  }
  
  if ((rc_ctrl.key & KEY_PRESSED_OFFSET_F) && top_mode_flag)
  {
    top_mode_flag = 0;

    if (keyboard_mode_up == 1)
    {
      keyboard_mode_up = 0;
      keyboard_mode_down = 0;
      keyboard_mode_mid = 1;
    }
    else
    {
      keyboard_mode_up = 1;
      keyboard_mode_down = 0;
      keyboard_mode_mid = 0;
    }
  }

  // 云台跟随模式切换 - C键
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_C)
  {
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    keyboard_mode_down = 0;
    keyboard_mode_up = 0;
    keyboard_mode_mid = 1;
  }

  // 当云台在某些模式下（如初始化），底盘不动
  if (gimbal_cmd_to_chassis_stop())
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
  }
}

/**
 * @brief 设置控制量 - 根据不同底盘控制模式调用不同的控制函数
 * @param[in] chassis_move_rc_to_vector 底盘所有信息
 * @retval none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
    chassis_follow_gimbal_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
    chassis_top_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL)
  {
    chassis_no_follow_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(chassis_move_rc_to_vector);
  }
  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);
  }
}

/**
 * @brief 底盘不移动控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief 底盘跟随云台控制 - 二维向量旋转算法（借鉴东莞理工学院 2024.02.29）
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  // 遥控器的通道值以及键盘按键得出速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);

  // 二维向量旋转算法
  {
    float SpinTop_Angle = 0;
    SpinTop_Angle = (gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd + 1200) % 8192 / 22.7555556f; // 22.7555556f=8192/360, 编码值转为角度值
    if (SpinTop_Angle > 360)
    {
      SpinTop_Angle = (SpinTop_Angle - 360) * 0.0174532f; // 2*pi/360 角度值转弧度值
    }
    else
    {
      SpinTop_Angle *= 0.0174532f;
    }

    chassis_move_rc_to_vector->vx_set = vx_set * cos(SpinTop_Angle) + vy_set * sin(SpinTop_Angle);
    chassis_move_rc_to_vector->vy_set = -vx_set * sin(SpinTop_Angle) + vy_set * cos(SpinTop_Angle);
  }

  // 设置控制相对云台角度
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set);
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle - GIMBAL_YAW_OFFSET_ECD);
  
  // 计算旋转PID角速度
  chassis_move_rc_to_vector->wz_set = PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, 
                                                chassis_move_rc_to_vector->chassis_relative_angle, 
                                                chassis_move_rc_to_vector->chassis_relative_angle_set);

  // 速度限幅
  chassis_move_rc_to_vector->wz_set = float_constrain(chassis_move_rc_to_vector->wz_set, -10.0f, 10.0f);
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, 
                                                      chassis_move_rc_to_vector->vx_min_speed, 
                                                      chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, 
                                                      chassis_move_rc_to_vector->vy_min_speed, 
                                                      chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief 底盘小陀螺控制 - 二维向量旋转算法（借鉴东莞理工学院 2024.02.29）
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  
  float vx_set = 0.0f, vy_set = 0.0f;

  // 遥控器的通道值以及键盘按键得出速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);

  // 二维向量旋转算法
  {
    float SpinTop_Angle = 0;
    SpinTop_Angle = fmodf(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd - GIMBAL_YAW_TOP_OFFSET_ECD + 8192, 8192.0f) / 22.7555556f; // 编码值转为角度值
    if (SpinTop_Angle > 360)
    {
      SpinTop_Angle = (SpinTop_Angle - 360) * 0.0174532f; // 2*pi/360
    }
    else
    {
      SpinTop_Angle *= 0.0174532f;
    }

    chassis_move_rc_to_vector->vx_set = vx_set * cos(SpinTop_Angle) - vy_set * sin(SpinTop_Angle);
    chassis_move_rc_to_vector->vy_set = vx_set * sin(SpinTop_Angle) + vy_set * cos(SpinTop_Angle);
  }

  // 小陀螺速度切换 - V键
  if (!(rc_ctrl.key & KEY_PRESSED_OFFSET_V))
  {
    top_speed_toggle_flag = 1;
  }
  
  if ((rc_ctrl.key & KEY_PRESSED_OFFSET_V) && top_speed_toggle_flag)
  {
    top_speed_toggle_flag = 0;
    if (top_mode_speed == CHASSIS_TOP_SPEED)
    {
      top_mode_speed = 10;    // 高速
    }
    else
    {
      top_mode_speed = CHASSIS_TOP_SPEED;    // 低速
    }
  }

  // 设置小陀螺转速
  if (vx_set == 0 && vy_set == 0)
  {
    chassis_move_rc_to_vector->wz_set = top_mode_speed;
  }
  else
  {
    // 平移时小陀螺速度变慢，限制功率
    chassis_move_rc_to_vector->wz_set = 1.2 * CHASSIS_TOP_SPEED;
  }

  // 速度限幅（小陀螺时移动速度变慢）
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, 
                                                      chassis_move_rc_to_vector->top_min_speed, 
                                                      chassis_move_rc_to_vector->top_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, 
                                                      chassis_move_rc_to_vector->top_min_speed, 
                                                      chassis_move_rc_to_vector->top_max_speed);
}

/**
 * @brief 底盘不跟随角度控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);

  chassis_move_rc_to_vector->wz_set = wz_set;
  chassis_move_rc_to_vector->vx_set = float_constrain(vx_set, 
                                                      chassis_move_rc_to_vector->vx_min_speed, 
                                                      chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(vy_set, 
                                                      chassis_move_rc_to_vector->vy_min_speed, 
                                                      chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief 底盘开环控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_move_rc_to_vector->vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->vy_set = -rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->wz_set = -rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
}

/**
 * @brief 底盘无力控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}
