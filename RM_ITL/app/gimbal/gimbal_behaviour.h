/**
 * @file    gimbal_behaviour.h
 * @brief   云台行为控制头文件
 * @details 包含云台行为模式设置和控制量设置的函数声明
 */

#ifndef __GIMBAL_BEHAVIOUR_H__
#define __GIMBAL_BEHAVIOUR_H__

/* =========================== 包含头文件 =========================== */
#include "gimbal.h"

/* =========================== 函数声明 =========================== */

/**
 * @brief 设置云台行为模式
 * @details 通过遥控器拨杆/按键判断，设置云台运动模式和自瞄标志位
 * @param[in,out] gimbal_mode_set 云台控制数据指针
 */
void gimbal_behaviour_mode_set(Gimbal_Ctrl_Cmd_s *gimbal_mode_set);

/**
 * @brief 设置云台控制量
 * @details 根据当前云台模式，将遥控器/鼠标输入映射到云台yaw和pitch控制参数
 * @param[in,out] gimbal_control_set 云台控制数据指针
 */
void gimbal_behaviour_control_set(Gimbal_Ctrl_Cmd_s *gimbal_control_set);

#endif /* __GIMBAL_BEHAVIOUR_H__ */
