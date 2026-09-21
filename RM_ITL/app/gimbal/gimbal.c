/**
 * @file    gimbal.c
 * @brief   云台控制主任务
 * @details 云台控制的主任务函数，负责模式切换、目标角度设置和电机更新
 *          PID计算和CAN发送由DJI6020_GimbalGyro_mode在motor_control回调中完成
 *          代码结构参考chassis模块的组织方式
 */

#include "gimbal.h"

#ifdef COMPILE_GIMBAL

#include "gimbal_behaviour.h"
#include "usbd_cdc_if.h"

/* =========================== 全局变量 =========================== */

/** 云台控制命令结构体实例 */
Gimbal_Ctrl_Cmd_s gimbal_cmd_control;
/** 云台测量数据（用于视觉系统） */
Gimbal_measure gimbal_measure_data;
/** 自瞄开关标志位 */
uint8_t auto_aim_flag = 0;
/** pitch角度值 */
float pitch_angle = 0.0f;

/* =========================== 静态变量 =========================== */

static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;
// 云台目标角度（度）
static float yaw_target = 0;
static float pitch_target = 0;
// 自瞄上次有效目标角度
static float yaw_target_last = 0;
static float pitch_target_last = 0;

/* =========================== 静态函数声明 =========================== */

static void gimbal_set_mode(void);
static void gimbal_set_control(void);
static void gimbal_control_loop(void);
static void gimbal_PID_switch(void);
static void gimbal_remote_target(void);
static void gimbal_auto_aiming_target(void);

/* =========================== 主任务函数 =========================== */

/**
 * @brief 云台主任务函数
 * @param argument 任务参数
 * @details 云台控制主循环：模式设置->控制量设置->PID切换->目标值设置->电机更新
 *          注意：PID计算不在此处进行，而是由DJI6020_GimbalGyro_mode在motor_control中完成
 */
void gimbal_task(void *argument)
{
    Gimbal_Init(Gimbal_6020);

    for(;;)
    {
        /* 设置云台控制模式 */
        gimbal_set_mode();
        /* 设置云台控制量（遥控器/鼠标映射） */
        gimbal_set_control();
        /* 手动/自瞄切换时更新PID参数 */
        gimbal_PID_switch();
        /* 检测模式切换（从零力到其他模式），清除PID积分器避免突变 */
        if(last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_control.gimbal_mode != GIMBAL_ZERO_FORCE)
        {
            Gimbal_PIDClear(Gimbal_6020);
        }
        last_mode = gimbal_cmd_control.gimbal_mode;
        /* 云台控制循环（设置目标值） */
        gimbal_control_loop();
        /* 更新电机状态（触发PID计算+CAN发送） */
        gimbal_motor_updata(Gimbal_6020);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* =========================== 初始化函数 =========================== */

/**
 * @brief 云台初始化函数
 * @param motors 电机实例数组指针
 * @details 初始化YAW轴和PITCH轴6020电机的串级PID控制器参数
 */
void Gimbal_Init(MotorInstance *motors)
{
    // YAW轴（偏航轴）6020电机PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop],
             1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner],
             YAW_MANUAL_INNER_KP, YAW_MANUAL_INNER_KI, YAW_MANUAL_INNER_KD,
             YAW_MANUAL_INNER_MAX_I, YAW_MANUAL_INNER_MAX_OUT, YAW_MANUAL_INNER_KFF);
    PID_Init(&motors[0].motor_data->pid[cascade_outer],
             YAW_MANUAL_OUTER_KP, YAW_MANUAL_OUTER_KI, YAW_MANUAL_OUTER_KD,
             YAW_MANUAL_OUTER_MAX_I, YAW_MANUAL_OUTER_MAX_OUT, YAW_MANUAL_OUTER_KFF);

    // PITCH轴（俯仰轴）6020电机PID初始化
    PID_Init(&motors[1].motor_data->pid[single_loop],
             1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner],
             PITCH_INNER_KP, PITCH_INNER_KI, PITCH_INNER_KD,
             PITCH_INNER_MAX_I, PITCH_INNER_MAX_OUT, PITCH_INNER_KFF);
    PID_Init(&motors[1].motor_data->pid[cascade_outer],
             PITCH_OUTER_KP, PITCH_OUTER_KI, PITCH_OUTER_KD,
             PITCH_OUTER_MAX_I, PITCH_OUTER_MAX_OUT, PITCH_OUTER_KFF);
}

/**
 * @brief 清除云台PID控制器积分项
 * @param motors 电机实例数组指针
 */
void Gimbal_PIDClear(MotorInstance *motors)
{
    if(motors == NULL)
    {
        return;
    }
    for(int i = 0; i < 2; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
}

/**
 * @brief 设置云台电机状态
 * @param status 电机状态（使能或失能）
 * @param motors 电机实例数组指针
 * @param motor_count 电机数量
 */
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
{
    switch (status)
    {
        case Motor_Enable:
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_enable(motors[i].motor_data);
            }
            break;
        case Motor_Disable:
        default:
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_disable(motors[i].motor_data);
            }
            break;
    }
}

/**
 * @brief 更新云台电机控制
 * @param motors 电机实例数组指针
 * @details 调用motor_control回调，触发DJI6020_GimbalGyro_mode执行PID计算和CAN发送
 */
void gimbal_motor_updata(MotorInstance *motors)
{
    if(motors == NULL)
    {
        return;
    }
    motors->motor_control(motors);
}

/* =========================== 控制流程函数 =========================== */

/**
 * @brief 设置云台控制模式
 */
static void gimbal_set_mode(void)
{
    gimbal_behaviour_mode_set(&gimbal_cmd_control);
}

/**
 * @brief 设置云台控制量（遥控器/鼠标映射）
 */
static void gimbal_set_control(void)
{
    gimbal_behaviour_control_set(&gimbal_cmd_control);
}

/**
 * @brief 云台控制循环 - 设置电机目标值
 * @details 根据当前模式设置 target_position（目标角度）和 target_current（重力补偿）
 *          PID计算由 DJI6020_GimbalGyro_mode 在 gimbal_motor_updata 中完成
 *
 *          数据流：
 *          gimbal_control_loop() 设置 target_position / target_current
 *              ↓
 *          gimbal_motor_updata() → DJI6020_GimbalGyro_mode()
 *              ↓
 *          PID_CascadeCalc(外环IMU角度, 内环陀螺仪角速度) + CAN发送
 */
static void gimbal_control_loop(void)
{
    switch (gimbal_cmd_control.gimbal_mode)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_NO_FOLLOW:
            // 零力/不跟随模式：关闭电机
            Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
            break;

        case GIMBAL_NORMAL:
        case GIMBAL_ABSOLUTE_ANGLE:
        {
            // 使能电机
            Gimbal_Motor_Status(Motor_Enable, Gimbal_6020, 2);

            // // 根据自瞄标志选择控制模式，计算目标角度
            // if(auto_aim_flag == AUTO_AIM_ON &&
            //    (vision_feedback.mode == 1 || vision_feedback.mode == 2))
            // {
            //     // 自瞄模式：使用视觉数据计算目标角度
            //     gimbal_auto_aiming_target();
            // }
            // else
            // {
                // 手动模式：使用遥控器/鼠标输入累积目标角度
                gimbal_remote_target();
            // }

            // 设置YAW轴目标角度（由DJI6020_GimbalGyro_mode用IMU做PID）
            Gimbal_6020[0].motor_data->target_position = yaw_target;
            Gimbal_6020[0].motor_data->target_current = 0;//附加电流

            // 设置PITCH轴目标角度 + 重力补偿
            Gimbal_6020[1].motor_data->target_position = pitch_target;
            Gimbal_6020[1].motor_data->target_current = Gravity_compensation(INS.Pitch);
            break;
        }

        default:
            Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
            break;
    }
}

/**
 * @brief PID参数切换（手动/自瞄模式）
 * @details 检测自瞄标志位变化，在手动和自瞄模式间切换不同的PID参数
 */
static void gimbal_PID_switch(void)
{
    static uint8_t last_auto_aim_flag = AUTO_AIM_OFF;
    if(auto_aim_flag != last_auto_aim_flag)
    {
        // 清除上次目标值和PID积分
        yaw_target_last = 0;
        pitch_target_last = 0;
        PID_Clear(Gimbal_6020[0].motor_data->pid);
        PID_Clear(Gimbal_6020[1].motor_data->pid);

        if(auto_aim_flag == AUTO_AIM_ON)
        {
            // 切换到自瞄PID参数
            PID_Init(&Gimbal_6020[0].motor_data->pid[single_loop],
                     1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&Gimbal_6020[0].motor_data->pid[cascade_inner],
                     YAW_AUTO_INNER_KP, YAW_AUTO_INNER_KI, YAW_AUTO_INNER_KD,
                     YAW_AUTO_INNER_MAX_I, YAW_AUTO_INNER_MAX_OUT, YAW_AUTO_INNER_KFF);
            PID_Init(&Gimbal_6020[0].motor_data->pid[cascade_outer],
                     YAW_AUTO_OUTER_KP, YAW_AUTO_OUTER_KI, YAW_AUTO_OUTER_KD,
                     YAW_AUTO_OUTER_MAX_I, YAW_AUTO_OUTER_MAX_OUT, YAW_AUTO_OUTER_KFF);

            PID_Init(&Gimbal_6020[1].motor_data->pid[single_loop],
                     1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&Gimbal_6020[1].motor_data->pid[cascade_inner],
                     PITCH_INNER_KP, PITCH_INNER_KI, PITCH_INNER_KD,
                     PITCH_INNER_MAX_I, PITCH_INNER_MAX_OUT, PITCH_INNER_KFF);
            PID_Init(&Gimbal_6020[1].motor_data->pid[cascade_outer],
                     PITCH_OUTER_KP, PITCH_OUTER_KI, PITCH_OUTER_KD,
                     PITCH_OUTER_MAX_I, PITCH_OUTER_MAX_OUT, PITCH_OUTER_KFF);
        }
        else
        {
            // 切换到手动PID参数
            PID_Init(&Gimbal_6020[0].motor_data->pid[single_loop],
                     1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&Gimbal_6020[0].motor_data->pid[cascade_inner],
                     YAW_MANUAL_INNER_KP, YAW_MANUAL_INNER_KI, YAW_MANUAL_INNER_KD,
                     YAW_MANUAL_INNER_MAX_I, YAW_MANUAL_INNER_MAX_OUT, YAW_MANUAL_INNER_KFF);
            PID_Init(&Gimbal_6020[0].motor_data->pid[cascade_outer],
                     YAW_MANUAL_OUTER_KP, YAW_MANUAL_OUTER_KI, YAW_MANUAL_OUTER_KD,
                     YAW_MANUAL_OUTER_MAX_I, YAW_MANUAL_OUTER_MAX_OUT, YAW_MANUAL_OUTER_KFF);

            PID_Init(&Gimbal_6020[1].motor_data->pid[single_loop],
                     1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&Gimbal_6020[1].motor_data->pid[cascade_inner],
                     PITCH_INNER_KP, PITCH_INNER_KI, PITCH_INNER_KD,
                     PITCH_INNER_MAX_I, PITCH_INNER_MAX_OUT, PITCH_INNER_KFF);
            PID_Init(&Gimbal_6020[1].motor_data->pid[cascade_outer],
                     PITCH_OUTER_KP, PITCH_OUTER_KI, PITCH_OUTER_KD,
                     PITCH_OUTER_MAX_I, PITCH_OUTER_MAX_OUT, PITCH_OUTER_KFF);
        }
        last_auto_aim_flag = auto_aim_flag;
    }
}

/* =========================== 目标角度计算函数 =========================== */

/**
 * @brief 遥控器手动模式 - 计算目标角度
 * @details 根据遥控器/鼠标输入累积目标角度，处理YAW回环和PITCH机械限位
 *          仅更新 yaw_target 和 pitch_target，不做PID
 */
static void gimbal_remote_target(void)
{
    /* ===== YAW轴 ===== */
    static uint8_t yaw_first_run = 1;
    if(yaw_first_run)
    {
        yaw_target = INS.Yaw;
        yaw_first_run = 0;
    }
    // 累积遥控器输入
    yaw_target += gimbal_cmd_control.yaw * YAW_SENSITIVITY;
    // 归一化到 -180~180
    while(yaw_target > 180.0f)  yaw_target -= 360.0f;
    while(yaw_target < -180.0f) yaw_target += 360.0f;

    /* ===== PITCH轴 ===== */
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run)
    {
        pitch_target = INS.Pitch;
        pitch_first_run = 0;
    }
    // 累积遥控器输入
    pitch_target += gimbal_cmd_control.pitch * PITCH_SENSITIVITY;
    // 机械限位保护
    if(pitch_target > PITCH_DOWN_MAX)
    {
        pitch_target = PITCH_DOWN_MAX;
    }
    else if(pitch_target < PITCH_UP_MAX)
    {
        pitch_target = PITCH_UP_MAX;
    }
}

/**
 * @brief 自瞄模式 - 计算目标角度
 * @details 使用视觉反馈数据设置目标角度，视觉丢失时保持上次有效位置
 *          仅更新 yaw_target 和 pitch_target，不做PID
 */
static void gimbal_auto_aiming_target(void)
{
    static uint8_t first_run = 1;

    // 检测视觉数据是否有效（yaw和pitch同时为0视为无效）
    uint8_t vision_valid = !(vision_feedback.yaw == 0.0f && vision_feedback.pitch == 0.0f);

    // 首次运行初始化目标角度
    if(first_run)
    {
        if(vision_valid)
        {
            yaw_target_last = vision_feedback.yaw * HUDU;
            pitch_target_last = vision_feedback.pitch * HUDU;
        }
        else
        {
            yaw_target_last = INS.Yaw;
            pitch_target_last = INS.Pitch;
        }
        yaw_target = yaw_target_last;
        pitch_target = pitch_target_last;
        first_run = 0;
    }

    if(vision_valid)
    {
        // 视觉有效：使用视觉发送的目标角度
        yaw_target = vision_feedback.yaw * HUDU;
        yaw_target_last = yaw_target;
        pitch_target = vision_feedback.pitch * HUDU;
        pitch_target_last = pitch_target;
    }
    else
    {
        // 视觉丢失：保持上次有效目标
        yaw_target = yaw_target_last;
        pitch_target = pitch_target_last;
    }

    // PITCH机械限位保护
    if(pitch_target > PITCH_DOWN_MAX)
    {
        pitch_target = PITCH_DOWN_MAX;
    }
    else if(pitch_target < PITCH_UP_MAX)
    {
        pitch_target = PITCH_UP_MAX;
    }
}

/* =========================== 工具函数 =========================== */

/**
 * @brief 编码器值转换为PITCH角度
 * @param motors PITCH轴电机实例指针
 * @return PITCH角度值（度）
 */
float Get_Pitch_Angle_From_Encoder(MotorInstance *motors)
{
    if(motors == NULL || motors->motor_data == NULL ||
       motors->motor_data->feedback == NULL)
    {
        return 0.0f;
    }
    float encoder_pos = motors[1].motor_data->feedback->pos;
    return (encoder_pos - 2348.0f) * 0.04515f - 26.0f;
}

/**
 * @brief pitch轴重力补偿（输出补偿电流）
 * @param pitch 当前pitch角度（度）
 * @return 补偿电流值
 */
float Gravity_compensation(float pitch)
{
    return 0.2465f * pitch * pitch + 39.274f * pitch - 5589.9f;
}

#endif // COMPILE_GIMBAL
