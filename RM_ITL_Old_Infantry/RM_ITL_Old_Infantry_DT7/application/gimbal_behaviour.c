
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include <math.h>

// 宏定义
#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief 遥控器的死区判断
 * @param input 输入的遥控器值
 * @param output 输出的死区处理后遥控器值
 * @param dealine 死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

// 全局变量定义
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
gimbal_behaviour_e last_gimbal_behaviour;

/* =========================== 变量声明 =========================== */
#ifdef vtm_rc_ctrl
  uint8_t gimbal_control_mode_index=0;
#endif
// 控制参数
extern gimbal_control_t gimbal_control;

/**
 * @brief 云台行为状态机设置
 * @param gimbal_mode_set 云台数据指针
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief 云台无力模式控制
 * @param yaw 发送yaw电机的原始值
 * @param pitch 发送pitch电机的原始值
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief 云台绝对角度控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief 云台相对角度控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief 云台静止控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief 云台自瞄模式控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);


/**
 * @brief 云台行为模式设置
 * @param gimbal_mode_set 云台数据指针
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    // 云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    // 根据云台行为状态机设置电机状态机
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:      // 无力模式
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
            break;

        case GIMBAL_INIT:            // 初始模式
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;

        case GIMBAL_ABSOLUTE_ANGLE:  // 绝对模式
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
            break;

        case GIMBAL_RELATIVE_ANGLE:  // 相对模式
        case GIMBAL_MOTIONLESS:       // 无运动模式
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;

        default:
            break;
    }
}

/**
 * @brief 云台行为控制
 * @param add_yaw 设置的yaw角度增加值，单位 rad
 * @param add_pitch 设置的pitch角度增加值，单位 rad
 * @param gimbal_control_set 云台数据指针
 */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static uint8_t last_behaviour = GIMBAL_ZERO_FORCE;

    // 根据云台行为模式选择对应的控制函数
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
            gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
            break;

        case GIMBAL_ABSOLUTE_ANGLE:
            gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
            break;

        case GIMBAL_RELATIVE_ANGLE:
            gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
            break;

        case GIMBAL_MOTIONLESS:
            gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
            break;

        case GIMBAL_AUTO://自瞄模式
            gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
            break;

        default:
            break;
    }

    // 模式切换时重置角度设定值
    if (last_behaviour != gimbal_behaviour)
    {
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    }

    last_behaviour = gimbal_behaviour;
}

/**
 * @brief 云台在某些行为下需要底盘停止
 * @return 1: 停止 0: 正常
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS) ? 1 : 0;
}

/**
 * @brief 云台在某些行为下需要射击停止
 * @return 1: 停止 0: 正常
 */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE) ? 1 : 0;
}

/**
 * @brief 云台行为状态机设置
 * @param gimbal_mode_set 云台数据指针
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    #ifdef DT7_rc_ctrl
        // 开关控制云台状态
        if (switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL]))  // 上
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        }
        else if (switch_is_mid(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL]))  // 中
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        }
        else if (switch_is_down(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL]))  // 下
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    #endif

    #ifdef i6x_rc_ctrl
        // 开关控制云台状态
        if (i6x_switch_is_down(i6x_ctrl.s[GIMBAL_MODE_CHANNEL]))  // 下
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        }
        else if (i6x_switch_is_mid(i6x_ctrl.s[GIMBAL_MODE_CHANNEL]))  // 中
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        }
        else if (i6x_switch_is_up(i6x_ctrl.s[GIMBAL_MODE_CHANNEL]))  // 上
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
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
            gimbal_control_mode_index = (gimbal_control_mode_index + 1) % 3; // 3种模式

            // 根据索引设置底盘模式
            switch (gimbal_control_mode_index)
            {
                case 0:
                    gimbal_behaviour = GIMBAL_ZERO_FORCE;
                    break;
                case 1:
                    gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
                    break;
                case 2:
                    gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
                    break;
            }
            last_gimbal_behaviour = gimbal_behaviour;
        }
        // 更新上一次R键状态
        r_key_last_state = r_key_current_state;

        if (open_fire(vtm_rc_data.mouse_right))
        {
            gimbal_behaviour = GIMBAL_AUTO;
        }
        else
        {
            gimbal_behaviour = last_gimbal_behaviour;
        }
    #endif

    // 遥控器错误时进入无力模式
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
}

/**
 * @brief 云台无力模式控制
 * @param yaw 发送yaw电机的原始值
 * @param pitch 发送pitch电机的原始值
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief 云台绝对角度控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    #ifdef DT7_rc_ctrl    
        rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
        rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    #endif

    #ifdef i6x_rc_ctrl
        rc_deadband_limit(i6x_ctrl.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
        rc_deadband_limit(i6x_ctrl.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    #endif

    #ifdef vtm_rc_ctrl
        rc_deadband_limit(vtm_rc_data.mouse_x, yaw_channel, RC_DEADBAND);
        rc_deadband_limit(vtm_rc_data.mouse_y, pitch_channel, RC_DEADBAND);
    #endif


    // 计算基础角度增量
    *yaw = yaw_channel * YAW_RC_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN;

}

/**
 * @brief 云台相对角度控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    #ifdef DT7_rc_ctrl    
        rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
        rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    #endif

    #ifdef i6x_rc_ctrl
        rc_deadband_limit(i6x_ctrl.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
        rc_deadband_limit(i6x_ctrl.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    #endif

    *yaw = yaw_channel * YAW_RC_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN ;
}

/**
 * @brief 云台静止控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief 云台自瞄模式控制
 * @param yaw yaw轴角度控制，为角度的增量 单位 rad
 * @param pitch pitch轴角度控制，为角度的增量 单位 rad
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // 检测是否受到上位机控制信号
    if (toe_is_error(USER_USART_DATA_TOE))
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
    else
    {
        // 上位机控制逻辑
        *yaw = auto_shoot.yaw_add;
        *pitch = auto_shoot.pitch_add;
    }
}

