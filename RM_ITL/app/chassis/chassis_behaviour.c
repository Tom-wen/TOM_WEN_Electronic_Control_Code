#include "chassis_behaviour.h"

/* =========================== 变量声明 =========================== */
#ifdef vtm_rc_ctrl
  uint8_t chassis_control_mode_index=0;
#endif
  uint8_t chassis_top_level=0;//默认0为低速小陀螺
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

/* ==================== 静态函数声明 ==================== */

/**
 * @brief 底盘无力控制 - 底盘模式为raw，将设定值都设置为0
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_zero_force_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector);


/**
 * @brief 底盘跟随云台控制 - 底盘跟随云台角度，根据角度差计算旋转角速度
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_follow_gimbal_yaw_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector);

/**
 * @brief 底盘小陀螺控制 - 一边旋转一边以云台指向方向运动
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_top_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector);

/**
 * @brief 底盘不跟随角度控制 - 底盘不跟随角度，旋转速度由参数直接设定
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_follow_yaw_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector);

/* ==================== 函数实现 ==================== */

/**
 * @brief 根据遥控器通道值，计算纵向和横移速度
 * @param[out] vx_set 纵向速度指针
 * @param[out] vy_set 横向速度指针
 * @param[in] chassis_move_rc_to_vector 底盘数据指针
 * @retval none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel = 0, vy_channel = 0;
  float vx_set_channel=0.0f, vy_set_channel=0.0f;

#ifdef DT7_rc_ctrl   
  // 死区限制，因为遥控器可能存在差异，摇杆在中间其值不为0
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
#endif
#ifdef i6x_rc_ctrl
    // 死区限制，因为遥控器可能存在差异，摇杆在中间其值不为0
  rc_deadband_limit(i6x_ctrl.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(i6x_ctrl.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
#endif
#ifdef vtm_rc_ctrl
  //键盘控制
  if(vtm_rc_data.key & KEY_PRESSED_OFFSET_SHIFT)//疾跑
  {
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_W)
      {
        vx_channel = KEY_MAX_X_SPEED;
      }
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_S)
      {
        vx_channel = -KEY_MAX_X_SPEED;
      }

      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_D)
      {
        vy_channel = KEY_MAX_Y_SPEED;
      }
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_A)
      {
        vy_channel = -KEY_MAX_Y_SPEED;
      }
  }
  else
  {
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_W)
      {
        vx_channel = KEY_POSITIVE_X_SPEED;
      }
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_S)
      {
        vx_channel = KEY_NEGATIVE_X_SPEED;
      }

      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_D)
      {
        vy_channel = KEY_POSITIVE_Y_SPEED;
      }
      if (vtm_rc_data.key & KEY_PRESSED_OFFSET_A)
      {
        vy_channel = KEY_NEGATIVE_Y_SPEED;
      }
  }


  //下面是图传遥控器
  if(vtm_rc_data.mode_sw == 1 || vtm_rc_data.mode_sw == 2)
  {
    rc_deadband_limit(vtm_rc_data.ch[1], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(vtm_rc_data.ch[0], vy_channel, CHASSIS_RC_DEADLINE);
  }

#endif

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

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
void chassis_behaviour_mode_set(Chassis_Ctrl_Cmd_s *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }


  #ifdef DT7_rc_ctrl 
    // 遥控器设置模式（非必要不建议改）
    if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 上挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
    }
    else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 中挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
    }
    else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 下挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
    }
  #endif

  #ifdef i6x_rc_ctrl
      // 遥控器设置模式（非必要不建议改）
    if (i6x_switch_is_down(i6x_ctrl.s[CHASSIS_MODE_CHANNEL])) // 下挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_TOP;
    }
    else if (i6x_switch_is_mid(i6x_ctrl.s[CHASSIS_MODE_CHANNEL])) // 中挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    else if (i6x_switch_is_up(i6x_ctrl.s[CHASSIS_MODE_CHANNEL])) // 上挡
    {
      chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
    }
  #endif
  #ifdef vtm_rc_ctrl
      // 检测R键是否按下（上升沿触发）
      static uint8_t r_key_last_state = 0;
      uint8_t r_key_current_state = (vtm_rc_data.key & KEY_PRESSED_OFFSET_R) ? 1 : 0;

      // 上升沿检测：当前按下且上次未按下
      if (r_key_current_state && !r_key_last_state)
      {
          // 切换到下一个模式（循环切换）
          chassis_control_mode_index = (chassis_control_mode_index + 1) % 2; // 2种模式

          // 根据索引设置底盘模式
          switch (chassis_control_mode_index)
          {
              case 0:
                  chassis_move_mode->chassis_mode = CHASSIS_TOP;
                  break;
              case 1:
                  chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
                  break;
          }
      }

      // 更新上一次R键状态
      r_key_last_state = r_key_current_state;
      if(vtm_rc_data.key & KEY_PRESSED_OFFSET_X)
      {
          chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
      }



      //下面是图传遥控器的相关控制
      if(vtm_rc_data.mode_sw == 1)
      {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
      }
      else if ( vtm_rc_data.mode_sw == 2)
      {
        chassis_move_mode->chassis_mode = CHASSIS_TOP;
      }
  #endif
}

/**
 * @brief 设置控制量 - 根据不同底盘控制模式调用不同的控制函数
 * @param[in] chassis_move_rc_to_vector 底盘所有信息
 * @retval none
 */
void chassis_behaviour_control_set(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
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
  else
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);
  }
}


/**
 * @brief 底盘跟随云台控制 - 二维向量旋转算法（借鉴东莞理工学院 2024.02.29）
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_follow_gimbal_yaw_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
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
    SpinTop_Angle = fmodf((Gimbal_6020->motor_data->feedback->pos + 1200.0), 8192.0f) / 22.7555556f; // 22.7555556f=8192/360, 编码值转为角度值
    if (SpinTop_Angle > 360)
    {
      SpinTop_Angle = (SpinTop_Angle - 360) * 0.0174532f; // 2*pi/360 角度值转弧度值
    }
    else
    {
      SpinTop_Angle *= 0.0174532f;
    }

    SpinTop_Angle += 240.0f * 0.0174532f;
    chassis_move_rc_to_vector->vx_set = vx_set * cos(SpinTop_Angle) + vy_set * sin(SpinTop_Angle);
    chassis_move_rc_to_vector->vy_set = -vx_set * sin(SpinTop_Angle) + vy_set * cos(SpinTop_Angle);
  }

  // 设置控制相对云台角度
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set);
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(motor_ecd_to_angle_change(Gimbal_6020->motor_data->feedback->pos, GIMBAL_YAW_OFFSET_ECD) - GIMBAL_YAW_OFFSET_ECD);
  
  // 计算旋转PID角速度
  chassis_move_rc_to_vector->w_set = PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, 
                                                chassis_move_rc_to_vector->chassis_relative_angle, 
                                                chassis_move_rc_to_vector->chassis_relative_angle_set);

  // 速度限幅
  chassis_move_rc_to_vector->w_set = float_constrain(chassis_move_rc_to_vector->w_set, -CHASSIS_FOLLOW_SPEED, CHASSIS_FOLLOW_SPEED );
}

/**
 * @brief 底盘小陀螺控制 - 二维向量旋转算法（借鉴东莞理工学院 2024.02.29）
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_top_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
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
    SpinTop_Angle = fmodf(Gimbal_6020->motor_data->feedback->pos - GIMBAL_YAW_TOP_OFFSET_ECD + 8192, 8192.0f) / 22.7555556f; // 编码值转为角度值
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

  // 设置小陀螺转速

    // 小陀螺挡位切换逻辑 (按下 Z 键切换)
  // 小陀螺挡位切换逻辑 (按下 Z 键切换)

  
  #ifdef vtm_rc_ctrl
  static uint8_t z_key_last_state = 0;
  static uint8_t chassis_top_gear_index = 0; 
  uint8_t z_key_current_state = (vtm_rc_data.key & KEY_PRESSED_OFFSET_Z) ? 1 : 0;
  
  if (z_key_current_state && !z_key_last_state)
  {
      chassis_top_gear_index = 1 - chassis_top_gear_index;
  }
  z_key_last_state = z_key_current_state;
  #endif


  // 设置小陀螺转速 - 根据挡位
  #ifdef vtm_rc_ctrl
  if(chassis_top_gear_index == 0)
  {
    if (vx_set == 0 && vy_set == 0)
    {
      chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED_GEAR_LOW;
    }
    else
    {
      chassis_move_rc_to_vector->wz_set = 0.7f * CHASSIS_TOP_SPEED_GEAR_LOW;
    }
    chassis_top_level = 0;
  }
  else
  {
    chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED_GEAR_HIGH;
    chassis_top_level = 1;
  }
  #else
  // 非 vtm 遥控器保持原有逻辑
  if (vx_set == 0 && vy_set == 0)
  {
    chassis_move_rc_to_vector->w_set = CHASSIS_TOP_SPEED;
  }
  else
  {
    chassis_move_rc_to_vector->w_set = 1.2f * CHASSIS_TOP_SPEED;
  }
  #endif

}

/**
 * @brief 底盘不跟随角度控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_no_follow_yaw_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);

  chassis_move_rc_to_vector->w_set = wz_set;
}


/**
 * @brief 底盘无力控制
 * @param[in] chassis_move_rc_to_vector 底盘数据
 * @retval 返回空
 */
static void chassis_zero_force_control(Chassis_Ctrl_Cmd_s *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->w_set = 0.0f;
}
