
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================== 包含头文件 =========================== */
#include "struct_typedef.h"
#include "chassis_task.h"

/* =========================== 遥控器通道定义 =========================== */
/**
 * @defgroup Chassis_RC_Channel 底盘遥控器通道定义
 * @brief 定义底盘控制所使用的遥控器通道
 * @{
 */

/** 前后方向遥控器通道号 */
#define CHASSIS_X_CHANNEL 1

/** 左右方向遥控器通道号 */
#define CHASSIS_Y_CHANNEL 0

/** 特殊模式下控制旋转的遥控器通道号 */
#define CHASSIS_WZ_CHANNEL 2

/** 底盘模式选择开关通道号 */
#define CHASSIS_MODE_CHANNEL 2

/** @} */

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

/** @} */

/* =========================== 外部变量声明 =========================== */
/**
 * @defgroup Chassis_External_Variables 底盘外部变量
 * @brief 键盘模式控制变量
 * @{
 */

extern uint8_t keyboard_mode_up;    /**< 上档位键盘模式标志 */
extern uint8_t keyboard_mode_mid;   /**< 中档位键盘模式标志 */
extern uint8_t keyboard_mode_down;  /**< 下档位键盘模式标志 */

/** @} */

/* =========================== 函数声明 =========================== */
/**
 * @defgroup Chassis_Behaviour_Functions 底盘行为控制函数
 * @brief 底盘模式设置和控制相关的函数接口
 * @{
 */

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
