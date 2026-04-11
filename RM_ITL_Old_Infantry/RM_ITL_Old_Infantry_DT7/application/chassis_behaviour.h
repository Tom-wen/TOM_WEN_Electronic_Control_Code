
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================== 包含头文件 =========================== */
#include "struct_typedef.h"
#include "chassis_task.h"

extern uint8_t chassis_top_level;
/* =========================== 函数声明 =========================== */
/**
 * @brief 设置底盘行为模式
 * @details 通过逻辑判断，将底盘设置为相应的运动模式
 * @param[in] chassis_move_mode: 底盘运动数据指针，包含底盘状态信息
 * @retval none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief 设置底盘控制量
 * @details 根据不同底盘控制模式，设置相应的运动控制参数
 *          三个参数控制不同方向的运动，函数内部会调用相应的控制函数
 * @param[out] vx_set: 纵向速度设定值，通常控制前后移动
 * @param[out] vy_set: 横向速度设定值，通常控制左右移动
 * @param[out] wz_set: 旋转角速度设定值，通常控制旋转运动
 * @param[in] chassis_move_rc_to_vector: 底盘运动数据结构体，包含底盘所有信息
 * @retval none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_BEHAVIOUR_H */
