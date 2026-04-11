#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "gimbal.h"

typedef enum
{
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_INIT,
  GIMBAL_ABSOLUTE_ANGLE,
  GIMBAL_RELATIVE_ANGLE,
  GIMBAL_MOTIONLESS,
  GIMBAL_AUTO,
  GIMBAL_POLE
} gimbal_behaviour_e;


/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_mode_set(Gimbal_Ctrl_Cmd_s *gimbal_mode_set);

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, Gimbal_Ctrl_Cmd_s *gimbal_control_set);


extern gimbal_behaviour_e gimbal_behaviour;


#endif
