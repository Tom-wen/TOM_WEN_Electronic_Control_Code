/**
 * @file    gimbal_behaviour.c
 * @brief   云台行为控制实现
 * @details 实现云台模式选择和遥控器输入到云台控制参数的映射
 *          代码结构参考chassis_behaviour.c的组织方式
 */

#include "gimbal_behaviour.h"

#ifdef COMPILE_GIMBAL

/* =========================== 宏定义 =========================== */

/**
 * @brief 遥控器死区判断
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

/* =========================== 外部变量 =========================== */

extern uint8_t auto_aim_flag;

/* =========================== 静态函数声明 =========================== */

/**
 * @brief 云台无力控制 - 所有控制参数设为0
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_zero_force_control(Gimbal_Ctrl_Cmd_s *gimbal_control_set);

/**
 * @brief 云台正常控制 - 遥控器/鼠标映射到yaw、pitch控制量
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_normal_control(Gimbal_Ctrl_Cmd_s *gimbal_control_set);

/* =========================== 函数实现 =========================== */

/**
 * @brief 云台模式选择
 * @param gimbal_mode_set 云台控制数据指针
 * @details 根据遥控器拨杆/按键状态设置云台控制模式和自瞄标志位
 */
void gimbal_behaviour_mode_set(Gimbal_Ctrl_Cmd_s *gimbal_mode_set)
{
    if(gimbal_mode_set == NULL)
    {
        return;
    }

    // 遥控器不在线时设置为零力模式
    if(sbus_online == 0)
    {
        gimbal_mode_set->gimbal_mode = GIMBAL_ZERO_FORCE;
        return;
    }

#ifdef DJI_REMOTE
    // DJI遥控器：根据左拨杆位置设置模式
    switch (left_switch)
    {
        case switch_down:
            gimbal_mode_set->gimbal_mode = GIMBAL_ZERO_FORCE;
            break;
        case switch_mid:
            gimbal_mode_set->gimbal_mode = GIMBAL_NORMAL;
            break;
        case switch_up:
            gimbal_mode_set->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;
        default:
            break;
    }
    // 侧拨杆控制自瞄开关
    if(side_switch_on)
    {
        auto_aim_flag = 1;
    }
    else if(side_switch_off)
    {
        auto_aim_flag = 0;
    }
#endif

#ifdef FS_REMOTE
    // FS遥控器：根据右拨杆位置设置模式
    switch (right_switch)
    {
        case switch_up:
            gimbal_mode_set->gimbal_mode = GIMBAL_ZERO_FORCE;
            break;
        case switch_mid:
            gimbal_mode_set->gimbal_mode = GIMBAL_NORMAL;
            break;
        case switch_down:
            gimbal_mode_set->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;
        default:
            break;
    }
    // 左拨杆控制自瞄开关
    switch (left_switch)
    {
        case switch_up:
            auto_aim_flag = 0;
            break;
        case switch_down:
            auto_aim_flag = 1;
            break;
        default:
            auto_aim_flag = 0;
            break;
    }
#endif

#ifdef VT_03_REMOTE
    {
        static uint8_t last_switch_state = 0xFF;
        static gimbal_mode_e current_gimbal_mode = GIMBAL_NORMAL;

        gimbal_mode_set->gimbal_mode = current_gimbal_mode;

        // R键循环切换模式（上升沿检测）
        if(rc_ctrl.key_rising_edge & (1 << 8))
        {
            if(current_gimbal_mode == GIMBAL_NORMAL)
            {
                current_gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            }
            else if(current_gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)
            {
                current_gimbal_mode = GIMBAL_NORMAL;
            }
            gimbal_mode_set->gimbal_mode = current_gimbal_mode;
        }
        // 拨杆切换模式（边沿检测，只有位置变化时才切换）
        else if((left_switch != last_switch_state) && (left_switch != switch_down))
        {
            switch (left_switch)
            {
                case switch_down:
                    current_gimbal_mode = GIMBAL_NO_FOLLOW;
                    break;
                case switch_mid:
                    current_gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
                    break;
                case switch_up:
                    current_gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
                    break;
                default:
                    current_gimbal_mode = GIMBAL_ZERO_FORCE;
                    break;
            }
            gimbal_mode_set->gimbal_mode = current_gimbal_mode;
            last_switch_state = left_switch;
        }
    }
#endif
}

/**
 * @brief 设置云台控制量
 * @param gimbal_control_set 云台控制数据指针
 * @details 根据当前云台模式调用不同的控制函数，将遥控器输入映射到yaw、pitch控制参数
 */
void gimbal_behaviour_control_set(Gimbal_Ctrl_Cmd_s *gimbal_control_set)
{
    if(gimbal_control_set == NULL)
    {
        return;
    }

    switch(gimbal_control_set->gimbal_mode)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_NO_FOLLOW:
            gimbal_zero_force_control(gimbal_control_set);
            break;

        case GIMBAL_NORMAL:
        case GIMBAL_ABSOLUTE_ANGLE:
            gimbal_normal_control(gimbal_control_set);
            break;

        default:
            gimbal_zero_force_control(gimbal_control_set);
            break;
    }
}

/* =========================== 静态控制函数 =========================== */

/**
 * @brief 云台无力控制
 * @param gimbal_control_set 云台数据指针
 */
static void gimbal_zero_force_control(Gimbal_Ctrl_Cmd_s *gimbal_control_set)
{
    gimbal_control_set->yaw = 0;
    gimbal_control_set->pitch = 0;
    gimbal_control_set->yaw_vel = 0;
    gimbal_control_set->pitch_vel = 0;
}

/**
 * @brief 云台正常/绝对角度控制（遥控器/鼠标输入映射）
 * @param gimbal_control_set 云台数据指针
 * @details 键盘鼠标有操作时优先使用鼠标，否则使用摇杆
 */
static void gimbal_normal_control(Gimbal_Ctrl_Cmd_s *gimbal_control_set)
{
#ifdef DJI_REMOTE
    // 键盘鼠标有操作时用鼠标，否则用摇杆
    if(rc_ctrl.mouse_x != 0 || rc_ctrl.mouse_y != 0)
    {
        gimbal_control_set->yaw = -rc_ctrl.mouse_x * 2.2f;
        gimbal_control_set->pitch = -rc_ctrl.mouse_y * 2.2f;
    }
    else
    {
        gimbal_control_set->yaw = -(float)rc_ctrl.rc.ch[2];
        gimbal_control_set->pitch = (float)rc_ctrl.rc.ch[3];
    }
#endif

#ifdef FS_REMOTE
    if(rc_ctrl.mouse_x != 0 || rc_ctrl.mouse_y != 0)
    {
        gimbal_control_set->yaw = -rc_ctrl.mouse_x * 2.2f;
        gimbal_control_set->pitch = -rc_ctrl.mouse_y * 2.2f;
    }
    else
    {
        gimbal_control_set->yaw = -(float)rc_ctrl.rc.ch[2];
        gimbal_control_set->pitch = (float)rc_ctrl.rc.ch[3];
    }
#endif

#ifdef VT_03_REMOTE
    if(rc_ctrl.mouse_x != 0 || rc_ctrl.mouse_y != 0)
    {
        gimbal_control_set->yaw = -rc_ctrl.mouse_x * 2.2f;
        gimbal_control_set->pitch = -rc_ctrl.mouse_y * 2.2f;
    }
    else
    {
        gimbal_control_set->yaw = -(float)rc_ctrl.rc.ch[2];
        gimbal_control_set->pitch = (float)rc_ctrl.rc.ch[3];
    }
#endif

    gimbal_control_set->yaw_vel = 0;
    gimbal_control_set->pitch_vel = 0;
}

#endif // COMPILE_GIMBAL
