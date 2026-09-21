#include "robotic_swing_gimbal.h"
#ifdef ROBOTIC_SWING_GIMBAL
/**
 * @brief 云台控制命令结构体
 * @details 用于存储云台控制的各种参数和命令
 */
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
/**
 * @brief 云台测量数据结构体
 * @details 用于存储云台当前的状态测量数据，传输给视觉系统
 */
Gimbal_measure gimbal_measure_data;

/**
 * @brief 自瞄开启关闭标志位
 * @details 1表示开启自瞄模式，0表示关闭自瞄模式
 */
uint8_t auto_aim_flag = 0;
// 静态变量：记录上一次有效的目标角度
static float yaw_target_last = 0;
static float pitch_target_last = 0;
// 统一的云台目标角度变量
static float yaw_target = 0;
static float pitch_target = 0;


static MPC_Preview3Dist yaw_autoaim_preview_mpc = {0};

static float normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 云台控制任务函数
 * @param[in] argument 任务参数指针
 * @details 这是云台控制的主任务函数，在FreeRTOS中周期性运行
 *          负责处理云台模式切换、控制逻辑执行和电机状态更新
 */
void gimbal_task(void *argument)
{
    // 初始化云台电机系统
    Gimbal_Init(Gimbal_6020);
    // 无限循环，持续执行云台控制逻辑
    for(;;)
    {
        // 处理遥控器输入数据，映射到云台控制命令
        RemoteControlGimbal(&gimbal_cmd_send);
        // 更新云台控制模式（根据遥控器拨杆状态）
        Gimbal_mode_update(&gimbal_cmd_send);
        //PID数据切换
        Gimbal_PID_Change(Gimbal_6020);
        // 根据当前云台模式执行相应的控制逻辑
        switch (gimbal_cmd_send.gimbal_mode)
        {
            case GIMBAL_ZERO_FORCE:
                // 零电流模式：关闭电机使能，电机不输出力矩
                Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
                yaw_target = INS.Yaw;
                pitch_target = INS.Pitch;
                break;
            
            case GIMBAL_NO_FOLLOW:
                // 零电流模式：关闭电机使能，电机不输出力矩
                Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
                yaw_target = INS.Yaw;
                pitch_target = INS.Pitch;
                break;  
                
            case GIMBAL_ABSOLUTE_ANGLE:
                // 绝对角度模式：使能电机并执行云台控制
                Gimbal_Motor_Status(Motor_Enable, Gimbal_6020, 2);
                Gimbal_control(Gimbal_6020, &gimbal_cmd_send, &vision_feedback, &INS);
                break;
        }
        
        // 更新云台电机控制（发送控制指令给电机）
        gimbal_motor_updata(Gimbal_6020);
        
        // 任务延时2ms，控制任务执行频率
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
/**
 * @brief 云台初始化函数
 * @param[in] motors 电机实例数组指针
 * @details 初始化云台电机的PID控制器参数
 *          包括单环PID、串级PID的内环和外环
 */
void Gimbal_Init(MotorInstance *motors)
{
    // YAW轴（偏航轴）6020电机PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 200.0f, 0.0f, 0.0f, 10.0f, 22000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 30.0f, 0.0f, 0.0f, 10.0f, 300.0f, 20.0f);

    //尝试使用adp控制YAW轴
    ADP_SetNorm(&motors[0].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
    //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
    ADP_Init(&motors[0].motor_data->adp[single_loop],
             5e-4f, 5e-5f,  0.93f,
             1000.0f, 0.05f, 150.0f,  0.01f,
             10.0f,   10000.0f, 0.0f,
             0.002f,            //gimbal_task 周期 2ms
             100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）

    //尝试Smc控制YAW轴
    SMC_Init(&motors[0].motor_data->smc, 10.0f, 80.0f, 0.0001f, 0.001f, 15000.0f, 0.8f, 0.5f);
    //C, K, epsilon, error_eps, u_max, J, delta
    //`C` 越大，位置误差在滑模面中的权重越高，系统更快把状态拉回滑模面
    //K增大后响应更快，但过大时可能带来振荡或电流抖动
    //epsilon增大后抗扰能力更强，同时也更容易带来抖振
    //`Sat(s, delta)` 的边界层厚度：
    // - `delta <= 0` 时，`Sat` 退化成 `sign`
    // - `delta > 0` 时，在边界层内用线性段代替硬切换，抖振更小
    // - 只有 `epsilon > 0` 时，`delta` 才真正影响主控制输出


    // PITCH轴（俯仰轴）6020电机PID初始化
    PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 45.0f, 0.0f, 0.0f, 300.0f, 16000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 38.0f, 0.0f, 0.0f, 0.0f, 165.0f, 0.0f);

        //尝试使用adp控制pitch轴
    ADP_SetNorm(&motors[1].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
    //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
    ADP_Init(&motors[1].motor_data->adp[single_loop],
             5e-5f, 5e-5f,  0.93f,
             1000.0f, 0.05f, 0.0f,  0.93f,
             10.0f,   10000.0f, 0.0f,
             0.002f,            //gimbal_task 周期 2ms
             100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）



    MPC_Init(&motors[0].motor_data->mpc[mpc_single_loop],
        15, 15,     //预测/控制时域
        1.0f, 0.002f,
        0.0f, 0.9837656f,  // 2×2 状态矩阵
        0.0f, -0.0010383f,   // 2×1 输入矩阵
        80000.0f, 80.0f , 0.0010f, 0.0f, // 权重 + 前馈
        3500.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间(与 gimbal_task 2ms 一致)
        0.1f); //死区

    MPC_Init(&motors[1].motor_data->mpc[mpc_single_loop],
        10, 10,     //预测/控制时域
        1.0f, 0.002f,
        0.0f, 0.9837656f,  // 2×2 状态矩阵
        0.0f, -0.0010383f,   // 2×1 输入矩阵
        10000.0f, 15.0f, 0.0018f, 0.0f, // 权重 + 前馈
        1000.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间
        0.02f); //死区

    /* YAW preview tracking MPC parameters tuned from baibi_test_10:
     * - The previous set improved dynamic tracking but introduced too much
     *   low-speed residual error / overshoot near target hold.
     * - This set pulls the delay compensation and control aggressiveness back
     *   to recover settling precision while keeping preview tracking enabled.
     */
  MPC_Preview3DistInitSimple(&yaw_autoaim_preview_mpc,
                             20, 10,
                             0.9920f, 0.0010383f,
                             2000.0f, 400.0f, 0.01f, 
                             0.0040f, 0.00320f,
                             25000.0f, 3000.0f,
                             0.002f, 0.0025f,
                              0.02f);
// 参数	含义	效果
// Q1	位置误差惩罚权重		不惜代价追位置
// Q2	速度误差惩罚权重		几乎不管速度超调 — 这就是速度失控的原因
// Q3	扰动状态惩罚	不惩罚
// R_u	控制量平方代价		太高的 u 会被惩罚
// R_delta	控制量变化代价		允许 u 快速变化（易震荡）
    MPC_Preview3DistClear(&yaw_autoaim_preview_mpc);
}
/**
 * @brief 清除云台PID控制器积分项
 * @param motors 云台电机实例数组指针
 * @details 在模式切换时调用，防止PID积分累积导致冲击
 */
void Gimbal_PIDClear(MotorInstance  *motors)
{
    if(motors == NULL)
    {
        return;
    }
    //云台6020
    for(int i = 0;i < 2; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
}
/**
 * @brief 云台电机状态设置函数
 * @param[in] status 电机状态（使能或失能）
 * @param[in] motors 电机实例数组指针
 * @param[in] motor_count 电机数量
 * @details 根据指定状态批量设置云台电机的使能状态
 */
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
{
    switch (status)
    {
        case Motor_Enable:
            // 使能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_enable(motors[i].motor_data);
            }
            break;
            
        case Motor_Disable:
            // 失能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_disable(motors[i].motor_data);
            }
            break;
            
        default:
            // 默认情况下失能所有电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_disable(motors[i].motor_data);
            }
            break;
    }
}
/**
 * @brief 云台模式更新函数
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @details 根据遥控器左拨杆状态更新云台控制模式
 */
void Gimbal_mode_update(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd)
{
    // 参数有效性检查
    if(Gimbal_Cmd == NULL)
    {
        return;
    }
    
    // 检查SBUS遥控器是否在线
    if(sbus_online == 0)
    {
        // 遥控器不在线时设置为零电流模式
        Gimbal_Cmd->gimbal_mode = GIMBAL_ZERO_FORCE;
        return;
    }
    
    #ifdef DJI_REMOTE
    // 根据左拨杆位置设置云台模式
    switch (left_switch)
    {
        case switch_down:
            // 拨杆向下：零电流模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ZERO_FORCE;
            break; 
            
        case switch_mid:
            // 拨杆中间：正常模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;
            
        case switch_up:
            // 拨杆向上：绝对角度模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;    
            
        default:
            break;
    }
    #endif
}

/**
 * @brief 遥控器数据映射函数
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @details 将遥控器通道数据映射到云台控制参数
 */
void RemoteControlGimbal(Gimbal_Ctrl_Cmd_s *Gimbal_Cmd) 
{
    // 参数有效性检查
    if(Gimbal_Cmd == NULL)
    {
        return;
    }
    
    // 根据当前云台模式映射遥控器数据到云台控制参数
    switch(Gimbal_Cmd->gimbal_mode) 
    {
        case GIMBAL_NORMAL:     
                // 正常模式：直接映射遥控器摇杆数据
            Gimbal_Cmd->yaw = -(float)rc_ctrl.rc.ch[2];     // YAW轴控制（左右旋转）
            Gimbal_Cmd->pitch = (float)rc_ctrl.rc.ch[3];    // PITCH轴控制（上下俯仰）
            Gimbal_Cmd->yaw_vel = 0;                        // YAW轴速度设为0
            Gimbal_Cmd->pitch_vel = 0;                      // PITCH轴速度设为0
            break;
            
        case GIMBAL_ABSOLUTE_ANGLE:             
            // 绝对角度模式：同样映射遥控器摇杆数据
                // 正常模式：直接映射遥控器摇杆数据
            Gimbal_Cmd->yaw = -(float)rc_ctrl.rc.ch[2];     // YAW轴控制（左右旋转）
            Gimbal_Cmd->pitch = (float)rc_ctrl.rc.ch[3];    // PITCH轴控制（上下俯仰）
            Gimbal_Cmd->yaw_vel = 0;                        // YAW轴速度设为0
            Gimbal_Cmd->pitch_vel = 0;                      // PITCH轴速度设为0
            break;
            
        case GIMBAL_ZERO_FORCE:     
            // 零电流模式：所有控制参数设为0
            Gimbal_Cmd->yaw = 0;        // YAW轴目标值为0
            Gimbal_Cmd->pitch = 0;      // PITCH轴目标值为0
            Gimbal_Cmd->yaw_vel = 0;    // YAW轴速度为0
            Gimbal_Cmd->pitch_vel = 0;  // PITCH轴速度为0
            break;
        
        case GIMBAL_NO_FOLLOW:     
            // 零电流模式：所有控制参数设为0
            Gimbal_Cmd->yaw = 0;        // YAW轴目标值为0
            Gimbal_Cmd->pitch = 0;      // PITCH轴目标值为0
            Gimbal_Cmd->yaw_vel = 0;    // YAW轴速度为0
            Gimbal_Cmd->pitch_vel = 0;  // PITCH轴速度为0
            break;
        default:
            break;
    }
}
/**
 * @brief 云台电机状态更新函数
 * @param[in] motors 电机实例数组指针
 * @details 调用电机控制函数，将控制指令发送给电机
 */
void gimbal_motor_updata(MotorInstance *motors)
{
    // 参数有效性检查
    if(motors == NULL)
    {
        return;
    }
    
    // 调用电机控制函数，更新电机状态
    motors[0].motor_control(&motors[0]);   // YAW → CAN2
    motors[1].motor_control(&motors[1]);   // PITCH → CAN1
}
/**
 * @brief 云台控制主函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] vision_feedback 视觉反馈数据指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 根据自瞄标志位选择控制模式（自瞄或手动）
 */
void Gimbal_control(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, VisionToGimbal *vision_feedback, INS_t *ins)
{
    #ifdef DJI_REMOTE
    // 根据侧拨杆状态更新自瞄标志位
    if(side_switch_on)
    {
        // 侧拨杆打开：开启自瞄模式
        auto_aim_flag = 1;
    }
    else if(side_switch_off)
    {
        // 侧拨杆关闭：关闭自瞄模式
        auto_aim_flag = 0;
    }
    #endif
    
    // 根据自瞄标志位选择控制模式
    switch (auto_aim_flag)
    {
        case AUTO_AIM_ON:
            // 自瞄模式开启：始终执行自动瞄准计算
            // 函数内部会根据 vision_feedback->mode 判断视觉是否有效
            Auto_aiming_cal(motors, Gimbal_Cmd, vision_feedback, ins);
            auto_aim_flag = 1;
            break;           
        case AUTO_AIM_OFF:
            // 自瞄模式关闭：执行遥控器手动控制
            remote_control(motors, Gimbal_Cmd, ins);
            auto_aim_flag = 0;
            break;
            
        default:
            break;
    }
}
/**
 * @brief 遥控器控制云台运动函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 实现基于遥控器输入的云台手动控制，包含PID控制和机械限位保护
 */
void remote_control(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, INS_t *ins)
{
    // YAW轴控制（偏航轴，左右旋转）
    float yaw_current = ins->Yaw;  // 范围在-180到180
    float yaw_target_rate_deg = 0.0f;
    static uint8_t yaw_first_run = 1;
    float yaw_target_accel=0.0f;    
    if(yaw_first_run == 1)
    {
        yaw_target = yaw_current;
        yaw_first_run = 0;
    }
    // 根据遥控器输入更新YAW目标角度
    float yaw_target_delta_deg = Gimbal_Cmd->yaw * yaw_sensitivity;
    yaw_target += yaw_target_delta_deg;
    yaw_target = normalize_angle_deg(yaw_target);
    float yaw_error = yaw_target - ins->Yaw;
    if (yaw_error > 180.0f)
    {
        yaw_target -= 360.0f;
    }
    else if (yaw_error < -180.0f)
    {
        yaw_target += 360.0f;
    }  

    yaw_target_rate_deg = yaw_target_delta_deg / AUTOAIM_REPLAY_TASK_DT_S;
    yaw_target_accel = yaw_target_rate_deg / AUTOAIM_REPLAY_TASK_DT_S;
    
    // // 执行YAW轴串级PID控制计算
    // // 参数：PID控制器、目标角度、当前角度、（角速度）
    // PID_CascadeCalc(motors[0].motor_data->pid, yaw_target, ins->Yaw, ins->Gyro[Zt] * hudu);
    // // 设置YAW轴电机目标电流
    // motors[0].motor_data->target_current = motors[0].motor_data->pid[cascade_inner].Out;

    // // 尝试使用ADP控制YAW轴
    // ADP_Calc(motors[0].motor_data->adp, yaw_target, ins->Yaw);
    // motors[0].motor_data->target_current = motors[0].motor_data->adp[single_loop].Out; 

    // 尝试使用SMC控制YAW轴
    SMC_SetRef(&motors[0].motor_data->smc, yaw_target);
    SMC_Tick(&motors[0].motor_data->smc, ins->Yaw, ins->Gyro[Zt] * hudu , 0.002);
    motors[0].motor_data->target_current = motors[0].motor_data->smc.u;

    // //尝试mpc控制yaw轴
    // MPC_CalcState(&motors[0].motor_data->mpc[mpc_single_loop],
    //               yaw_target,
    //               ins->Yaw,
    //               yaw_target_rate_deg - ins->Gyro[Zt] * hudu);
    // motors[0].motor_data->target_current = motors[0].motor_data->mpc[mpc_single_loop].Out;

    // /* Preview simple interface expects:
    //  * target_acc: rad/s^2
    //  * target_vel: rad/s
    //  * target_pos: deg
    //  * feedback_pos: deg
    //  * feedback_vel: rad/s
    //  * yaw_target_rate_deg is currently deg/s, so convert back to rad/s here.
    //  */

    
    // target_accel = yaw_target_accel ;
    
    // target_vel = yaw_target_rate_deg;
    // MPC_Preview3DistCalcSimple(&yaw_autoaim_preview_mpc,
    //                            0.1f,
    //                            target_vel,
    //                            yaw_target,
    //                            ins->Yaw,
    //                            ins->Gyro[Zt]);
    // out_put = yaw_autoaim_preview_mpc.Out;
    // motors[0].motor_data->target_current = yaw_autoaim_preview_mpc.Out;
    
    

    // PITCH轴控制（俯仰轴，上下运动）
    // 定义机械限位范围
    const float pitch_up_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_down_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    // 使用陀螺仪PITCH角度作为反馈值
    float pitch_current = ins->Pitch;  
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run == 1)
    {
        pitch_target = pitch_current;
        pitch_first_run = 0;
    } 
    // 根据遥控器输入更新PITCH目标角度
    pitch_target += Gimbal_Cmd->pitch * pitch_sensitivity;
    pitch_target =loop_fp32_constrain(pitch_target,-180.0f,180.0f);// 将角度约束到-180到180之间

    float pitch_err = pitch_target - pitch_current;
    if (pitch_err > 180.0f)
    {
        pitch_target -= 360.0f;
    }
    else if (pitch_err < -180.0f)
    {
        pitch_target += 360.0f;
    }  
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_down_limit) 
    { 
        // 超过下限则限制在下限值
        pitch_target = pitch_down_limit;
    }
    else if (pitch_target < pitch_up_limit)
    {
        // 超过上限则限制在上限值
        pitch_target = pitch_up_limit;
    }

    // 执行PITCH轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（X轴角速度）
    PID_CascadeCalc(motors[1].motor_data->pid, pitch_target, pitch_current, -ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标电流
    motors[1].motor_data->target_current = motors[1].motor_data->pid[cascade_inner].Out + GRAVITY_COMPENSATION ;

    // //尝试使用ADP控制PITCH轴
    // ADP_Calc(motors[1].motor_data->adp, pitch_target, ins->Pitch);
    // motors[1].motor_data->target_current = motors[1].motor_data->adp[single_loop].Out; 

    // //尝试mpc控制pitch轴
    // MPC_Calc(&motors[1].motor_data->mpc[mpc_single_loop], pitch_target, ins->Pitch);
    // motors[1].motor_data->target_current = motors[1].motor_data->mpc[mpc_single_loop].Out;
}
/**
 * @brief 自瞄解算函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] vision_feedback 视觉反馈数据指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 处理视觉系统反馈的数据，实现自动瞄准功能
 */
void Auto_aiming_cal(MotorInstance *motors, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, VisionToGimbal *vision_feedback, INS_t *ins)
{



    // 陀螺仪绝对角度闭环
    float yaw_current = ins->Yaw;  // 当前陀螺仪YAW角度（-180到180）
    float pitch_current = ins->Pitch;  // 获取当前PITCH角度
    // ========== PITCH轴控制 ==========
    // 定义机械限位范围
    const float pitch_up_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_down_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    
    // 检测视觉数据是否有效
    uint8_t vision_valid = 1;
    
    // 判断条件：如果yaw和pitch都是0，认为是无效数据
    // 注意：这里假设0,0表示视觉丢失
    if (vision_feedback->yaw == 0.0f && vision_feedback->pitch == 0.0f)
    {
        vision_valid = 0;
    }  

    if (vision_valid)
    {
        // 视觉有效：使用视觉发送的目标角度
        yaw_target = vision_feedback->yaw * hudu;
        yaw_target_last = yaw_target;  // 更新缓存
        pitch_target = vision_feedback->pitch * hudu;
        pitch_target_last = pitch_target;  // 更新缓存
    }
    else
    {
        // 视觉丢失：使用上一次的有效目标（保持位置）
        yaw_target = ins->Yaw;
        pitch_target =ins->Pitch;
    }
    
    // // 执行YAW轴串级PID控制计算
    // // 参数：PID控制器、目标角度、当前角度、（角速度）
    // PID_CascadeCalc(motors[0].motor_data->pid, yaw_target, ins->Yaw, ins->Gyro[Zt] * hudu);
    // // 设置YAW轴电机目标电流
    // motors[0].motor_data->target_current = motors[0].motor_data->pid[cascade_inner].Out;



    // // //三环pid分解：角度环、速度环、加速度环
    // float target_current_angle=0.0f;
    // float target_current_w=0.0f;
    // float target_current_a=0.0f;
    // // // 执行YAW轴串级PID控制计算
    // // 参数：PID控制器、目标角度、当前角度、（角速度）
    // PID_Calc(motors[0].motor_data->pid, yaw_target, ins->Yaw);
    // // 设置YAW轴电机目标电流
    // target_current_angle = motors[0].motor_data->pid[single_loop].Out;

    // // PID_Calc(motors[0].motor_data->pid, vision_feedback->yaw_vel, ins->Gyro[Zt]);
    // // target_current_w = motors[0].motor_data->pid[single_loop].Out;

    // // PID_Calc(motors[0].motor_data->pid, vision_feedback->yaw_acc, ins->Accel[Zt]);
    // // target_current_a = motors[0].motor_data->pid[single_loop].Out;

    // motors[0].motor_data->target_current = target_current_angle + target_current_w + target_current_a;



    //     // 尝试使用ADP控制YAW轴
    // float target_current_angle=0.0f;
    // float target_current_w=0.0f;
    // float target_current_a=0.0f;
    // ADP_Calc(motors[0].motor_data->adp, yaw_target, ins->Yaw);//角度
    // target_current_angle = motors[0].motor_data->adp[single_loop].Out; 
    // motors[0].motor_data->target_current = target_current_angle;


    // // MPC 状态: e = ref - fdb (度), de = d(ref-fdb)/dt = ref_rate - fdb_rate (度/秒)
    // 视觉 yaw_vel/yaw_acc 单位为 rad/s, rad/s², 需乘 hudu 与 ins->Gyro 单位对齐
    {
        float ref_rate_deg = vision_feedback->yaw_vel * hudu;
        float fdb_rate_deg = ins->Gyro[Zt] * hudu;
        float d_error_deg  = ref_rate_deg - fdb_rate_deg;

        MPC_CalcState(&motors[0].motor_data->mpc[mpc_single_loop],
                      yaw_target, ins->Yaw, d_error_deg);

        // 速度前馈已通过 state_dot 进入 MPC 预测, 此处仅叠加加速度前馈作为附加补偿
        float yaw_ff_current = vision_feedback->yaw_acc * AUTOAIM_YAW_ACC_FF_GAIN;

        if (yaw_ff_current > AUTOAIM_YAW_FF_CURRENT_LIMIT)
        {
            yaw_ff_current = AUTOAIM_YAW_FF_CURRENT_LIMIT;
        }
        else if (yaw_ff_current < -AUTOAIM_YAW_FF_CURRENT_LIMIT)
        {
            yaw_ff_current = -AUTOAIM_YAW_FF_CURRENT_LIMIT;
        }

        float yaw_cmd_current = motors[0].motor_data->mpc[mpc_single_loop].Out ;//+ yaw_ff_current;
        motors[0].motor_data->target_current = yaw_cmd_current;
    }


    // MPC_Preview3DistCalcSimple(&yaw_autoaim_preview_mpc,
    //                            vision_feedback->yaw_acc,
    //                            vision_feedback->yaw_vel,
    //                            yaw_target,
    //                            ins->Yaw,
    //                            ins->Gyro[Zt]);
    // out_put = yaw_autoaim_preview_mpc.Out;
    // motors[0].motor_data->target_current = yaw_autoaim_preview_mpc.Out;


    // // 尝试使用SMC控制YAW轴
    // SMC_SetRef(&motors[0].motor_data->smc, yaw_target);
    // SMC_Tick(&motors[0].motor_data->smc, ins->Yaw, ins->Gyro[Zt] * hudu , 0.002);
    // motors[0].motor_data->target_current = motors[0].motor_data->smc.u;



    pitch_target =loop_fp32_constrain(pitch_target,-180.0f,180.0f);// 将角度约束到-180到180之间
    float pitch_err = pitch_target - pitch_current;
    if (pitch_err > 180.0f)
    {
        pitch_target -= 360.0f;
    }
    else if (pitch_err < -180.0f)
    {
        pitch_target += 360.0f;
    }  
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_down_limit) 
    { 
        // 超过下限则限制在下限值
        pitch_target = pitch_down_limit;
    }
    else if (pitch_target < pitch_up_limit)
    {
        // 超过上限则限制在上限值
        pitch_target = pitch_up_limit;
    }

    // 执行PITCH轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（X轴角速度）
    PID_CascadeCalc(motors[1].motor_data->pid, pitch_target, pitch_current, -ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标电流
    motors[1].motor_data->target_current = motors[1].motor_data->pid[cascade_inner].Out + GRAVITY_COMPENSATION ;

    // ADP_Calc(motors[1].motor_data->adp, pitch_target, ins->Pitch);
    // motors[1].motor_data->target_current = motors[1].motor_data->adp[single_loop].Out; 

    // //尝试mpc控制pitch轴
    // MPC_Calc(&motors[1].motor_data->mpc[mpc_single_loop], pitch_target, ins->Pitch);
    // motors[1].motor_data->target_current = motors[1].motor_data->mpc[mpc_single_loop].Out;
}
void Gimbal_PID_Change(MotorInstance *motors)
{
    static uint8_t last_auto_aim_flag = AUTO_AIM_OFF;
    // 检测PID是否需要切换
    if(auto_aim_flag != last_auto_aim_flag)
    {
        if(auto_aim_flag == AUTO_AIM_ON)
        {
            //视觉上次变量清零：
            MPC_Clear(&motors[0].motor_data->mpc[mpc_single_loop]);
            MPC_Preview3DistClear(&yaw_autoaim_preview_mpc);

            // YAW轴（偏航轴）6020电机PID初始化
            PID_Init(&motors[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 22000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_outer], 100.0f, 0.0f, 0.1f, 10.0f, 1000.0f, 20.0f);
            // PITCH轴（俯仰轴）6020电机PID初始化
            PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_inner], 30.0f, 0.0f, 0.0f, 300.0f, 22000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_outer], 100.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f);

            //尝试Smc控制YAW轴
            SMC_Init(&motors[0].motor_data->smc, 10.0f, 80.0f, 0.0001f, 0.001f, 15000.0f, 0.8f, 0.5f);
            //C, K, epsilon, error_eps, u_max, J, delta
            //`C` 越大，位置误差在滑模面中的权重越高，系统更快把状态拉回滑模面
            //K增大后响应更快，但过大时可能带来振荡或电流抖动
            //epsilon增大后抗扰能力更强，同时也更容易带来抖振
            //`Sat(s, delta)` 的边界层厚度：
            // - `delta <= 0` 时，`Sat` 退化成 `sign`
            // - `delta > 0` 时，在边界层内用线性段代替硬切换，抖振更小
            // - 只有 `epsilon > 0` 时，`delta` 才真正影响主控制输出

                //尝试使用adp控制YAW轴
            ADP_SetNorm(&motors[0].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
            //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
            ADP_Init(&motors[0].motor_data->adp[single_loop],
                5e-5f, 5e-5f,  0.93f,
                1000.0f, 1.05f, 30.0f,  10.0f,
                10.0f,   10000.0f, 0.0f,
                0.001f,            //接收到视觉的频率
                100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）
                            //尝试使用adp控制pitch轴
            ADP_SetNorm(&motors[1].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
            //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
            ADP_Init(&motors[1].motor_data->adp[single_loop],
                5e-5f, 5e-5f,  0.93f,
                1000.0f, 0.05f, 40.0f,  10.0f,
                10.0f,   10000.0f, 0.0f,
                0.002f,            //gimbal_task 周期 2ms
                100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）

            MPC_Init(&motors[0].motor_data->mpc[mpc_single_loop],
                15, 15,     //预测/控制时域
                1.0f, 0.002f,
                0.0f, 0.9837656f,  // 2×2 状态矩阵
                0.0f, -0.0010383f,   // 2×1 输入矩阵
                500000.0f, 10.0f , 0.017f, 0.0f, // 权重 + 前馈//500000.0f, 10.0f
                3500.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间(与 gimbal_task 2ms 一致)
                0.1f); //死区

            MPC_Init(&motors[1].motor_data->mpc[mpc_single_loop],
                10, 10,     //预测/控制时域
                1.0f, 0.002f,
                0.0f, 0.9837656f,  // 2×2 状态矩阵
                0.0f, -0.0010383f,   // 2×1 输入矩阵
                10000.0f, 15.0f, 0.0018f, 0.0f, // 权重 + 前馈
                1000.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间(与 gimbal_task 2ms 一致)
                0.02f); //死区
            
        }
        else
        {
            //视觉上次变量清零：  
            MPC_Clear(&motors[0].motor_data->mpc[mpc_single_loop]);
            MPC_Preview3DistClear(&yaw_autoaim_preview_mpc);

            // YAW轴（偏航轴）6020电机PID初始化
            PID_Init(&motors[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_inner], 200.0f, 0.0f, 0.0f, 10.0f, 22000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_outer], 30.0f, 0.0f, 0.0f, 10.0f, 300.0f, 20.0f);

            //尝试Smc控制YAW轴
            SMC_Init(&motors[0].motor_data->smc, 10.0f, 80.0f, 0.0001f, 0.001f, 15000.0f, 0.8f, 0.5f);
            //C, K, epsilon, error_eps, u_max, J, delta
            //`C` 越大，位置误差在滑模面中的权重越高，系统更快把状态拉回滑模面
            //K增大后响应更快，但过大时可能带来振荡或电流抖动
            //epsilon增大后抗扰能力更强，同时也更容易带来抖振
            //`Sat(s, delta)` 的边界层厚度：
            // - `delta <= 0` 时，`Sat` 退化成 `sign`
            // - `delta > 0` 时，在边界层内用线性段代替硬切换，抖振更小
            // - 只有 `epsilon > 0` 时，`delta` 才真正影响主控制输出

            // PITCH轴（俯仰轴）6020电机PID初始化
            PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_inner], 45.0f, 0.0f, 0.0f, 300.0f, 16000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_outer], 38.0f, 0.0f, 0.0f, 0.0f, 165.0f, 0.0f);

                            //尝试使用adp控制YAW轴
            ADP_SetNorm(&motors[0].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
            //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
            ADP_Init(&motors[0].motor_data->adp[single_loop],
                5e-4f, 5e-5f,  0.93f,
                1000.0f, 0.05f, 150.0f,  0.01f,
                10.0f,   10000.0f, 0.0f,
                0.002f,            //gimbal_task 周期 2ms
                100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）
                            //尝试使用adp控制pitch轴
            ADP_SetNorm(&motors[1].motor_data->adp[single_loop], 0.1f, 200.0f, 100.0f, 500.0f);  // 让各状态 ~O(1)
            //      alpha_c alpha_a gamma   Q1      Q2    Q3    R     maxI  maxOut Kff     dt      Wc_max  Wa_max
            ADP_Init(&motors[1].motor_data->adp[single_loop],
                5e-5f, 5e-5f,  0.93f,
                1000.0f, 0.05f, 20.0f,  00.0f,
                10.0f,   10000.0f, 0.0f,
                0.002f,            //gimbal_task 周期 2ms
                100.0f, 100.0f);    //Wc_max, Wa_max（符号修正后可放宽）


            MPC_Init(&motors[0].motor_data->mpc[mpc_single_loop],
                15, 15,     //预测/控制时域
                1.0f, 0.002f,
                0.0f, 0.9837656f,  // 2×2 状态矩阵
                0.0f, -0.0010383f,   // 2×1 输入矩阵
                30000.0f, 80.0f , 0.0010f, 0.0f, // 权重 + 前馈
                3500.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间(与 gimbal_task 2ms 一致)
                0.08f); //死区

            MPC_Init(&motors[1].motor_data->mpc[mpc_single_loop],
                10, 10,     //预测/控制时域
                1.0f, 0.002f,
                0.0f, 0.9837656f,  // 2×2 状态矩阵
                0.0f, -0.0010383f,   // 2×1 输入矩阵
                10000.0f, 15.0f, 0.0018f, 0.0f, // 权重 + 前馈
                1000.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间
                0.02f); //死区
        }
        last_auto_aim_flag = auto_aim_flag; 
    }
}

/**
 * @brief 编码器值转换为PITCH角度
 * @param[in] motors PITCH轴电机实例指针
 * @return PITCH角度值（度）
 * @details 根据标定数据转换编码器值到角度：
 *          -26.0° 对应编码器 2348
 *           20.1° 对应编码器 3369
 *          转换系数：46.1° / 1021 = 0.04515°/单位
 */
float Get_Pitch_Angle_From_Encoder(MotorInstance *motors)
{
    // 参数检查
    if (motors == NULL || motors->motor_data == NULL || 
        motors->motor_data->feedback == NULL)
    {
        return 0.0f;
    }
    
    // 获取编码器原始值
    float encoder_pos = motors[1].motor_data->feedback->pos;
    
    // 转换为角度：offset为2348（对应-26度），系数为0.04515
    float pitch_angle = (encoder_pos - 2348.0f) * 0.04515f - 26.0f;
    
    return pitch_angle;
}



//循环约束
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue) {
        return Input;
    }
    float len = maxValue - minValue;
    Input = fmodf(Input - minValue, len);
    if (Input < 0) {
        Input += len;
    }
    return Input + minValue;
}

#endif 
