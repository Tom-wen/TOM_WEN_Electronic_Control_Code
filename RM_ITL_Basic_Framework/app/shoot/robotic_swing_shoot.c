#include "robotic_swing_shoot.h"
#include <math.h>

#ifdef ROBOTIC_SWING_SHOOT
//发射机构电机发送数据结构体实例
Shoot_Ctrl_Cmd_s shoot_cmd_send;
uint8_t fire_flag = 0;

//发射延迟测试量：从切换发射模式到弹丸被送出的耗时，单位us
float fire_latency_us = 0;

/**
 * @brief 发射机构主任务函数
 * @param argument 任务参数
 * @details 这是发射机构主任务函数，负责根据不同的模式控制发射机构
 */
void shoot_task(void *argument)
{
    Shoot_Init(Shoot_3508, Trigger_2006, &shoot_cmd_send);
    Upper_Computer_Init(&fire_latency_us);

    for(;;)
    {
        // 根据遥控器输入更新发射机构控制指令
        RemoteControlShoot(&shoot_cmd_send);
        Shoot_mode_update(&shoot_cmd_send);
        switch (shoot_cmd_send.shoot_mode)
        {
            case SHOOT_ZERO_FORCE:
                // 零力模式：关闭所有电机输出
                Shoot_Motor_Status(Motor_Disable, Shoot_3508, 2);
                Shoot_Motor_Status(Motor_Disable, Trigger_2006, 1);
                break;
            case SHOOT_CLOSE:
                // 摩擦轮正常工作模式 
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 2);
                Shoot_Motor_Status(Motor_Enable, Trigger_2006, 1);
                Shoot_Motor_Set(Shoot_3508, Trigger_2006, &shoot_cmd_send, &vision_feedback);   
                break; 
            case SHOOT_NORMAL:
                // 摩擦轮正常工作模式 
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 2);
                Shoot_Motor_Status(Motor_Enable, Trigger_2006, 1);
                Shoot_Motor_Set(Shoot_3508, Trigger_2006, &shoot_cmd_send, &vision_feedback);
                break;
            case FIRE_MODE:
                // 发射模式
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 2);
                Shoot_Motor_Status(Motor_Enable, Trigger_2006, 1);
                Shoot_Motor_Set(Shoot_3508, Trigger_2006, &shoot_cmd_send, &vision_feedback);
                break;                         
        }
        // 更新底盘电机状态
        shoot_motor_updata(Shoot_3508, Trigger_2006);
        // 测量从切换发射模式到弹丸送出的延迟
        Fire_Latency_Test(Shoot_3508, &shoot_cmd_send);
        usart_vofa_send(&huart7);
        // 控制任务执行频率（2ms周期）
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 发射机构初始化函数
 * @param motors 发射机构实例数组指针
 * @param Chassis_Cmd 发射机构控制命令结构体指针
 * @details 初始化发射机构各电机的PID控制器参数
 */
void Shoot_Init(MotorInstance *motors, MotorInstance *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd)
{   
    //发射机构3508单环PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 8.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 65.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], 8.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 65.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    //拨弹电机
    PID_Init(&motors2[0].motor_data->pid[single_loop], 23.0f, 0.0f, 0.0f, 10.0f, 20000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 50.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
}

/**
 * @brief 设置发射机构电机状态
 * @param status 电机状态（启用或禁用）
 * @param motors 发射机构电机实例数组指针
 * @param motor_count 电机数量
 */
void Shoot_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
{
    switch (status)
    {
    case Motor_Enable:
        // 启用所有指定电机
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_enable(motors[i].motor_data);
        }
        break;
    case Motor_Disable:
        // 禁用所有指定电机
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
        }
        break;
    default:
        // 默认情况下禁用所有电机
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
        }
        break;
    }
}

/**
 * @brief 更新发射机构工作模式
 * @param Chassis_Cmd 发射机构控制命令结构体指针
 * @details 根据遥控器开关状态更新发射机构工作模式
 */
void Shoot_mode_update(Shoot_Ctrl_Cmd_s *Shoot_Cmd)
{
    if(Shoot_Cmd == NULL)
    {
        return;
    }
    // 如果SBUS信号断开，则强制进入零力模式
    if(sbus_online == 0)
    {
        Shoot_Cmd->shoot_mode = SHOOT_ZERO_FORCE;
        return;
    }
    #ifdef DJI_REMOTE
    // 根据遥控器右侧开关位置选择发射机构模式
    switch (right_switch)
    {
    case switch_down:
        Shoot_Cmd->shoot_mode = SHOOT_CLOSE;
        break; 
    case switch_mid:
        if(left_switch == switch_mid || left_switch == switch_up)
        {
            Shoot_Cmd->shoot_mode = SHOOT_NORMAL;
            break;
        }
        else
        {
            Shoot_Cmd->shoot_mode = SHOOT_CLOSE;
            break;    
        }
    case switch_up:
        if(left_switch == switch_mid || left_switch == switch_up)
        {
            Shoot_Cmd->shoot_mode = FIRE_MODE;
            break;    
        }
        else
        {
            Shoot_Cmd->shoot_mode = SHOOT_CLOSE;
            break;    
        }
        break;    
    default:
        Shoot_Cmd->shoot_mode = SHOOT_ZERO_FORCE;
        break;
    }
    #endif

}

/**
 * @brief 更新发射机构电机控制
 * @param motors 发射机构电机实例数组指针
 * @details 执行电机控制算法并更新电机输出
 */
void shoot_motor_updata(MotorInstance *motors, MotorInstance *motors2)
{
    if(motors == NULL)
    {
        return;
    }
    //更新摩擦轮3508电机控制输出
    motors->motor_control(motors);
    motors2->motor_control(motors2);
}

/**
 * @brief 根据遥控器输入控制发射机构
 * @param Chassis_Cmd 发射机构控制命令结构体指针
 * @details 解析遥控器通道数据，计算发射机构在不同模式下的运动指令
 */
void RemoteControlShoot(Shoot_Ctrl_Cmd_s *Shoot_Cmd) 
{
    if(Shoot_Cmd == NULL)
    {
        return;
    }
    /***********************************确定发射机构电机的目标速度*****************************************/
    switch(Shoot_Cmd->shoot_mode) 
    {
        case SHOOT_ZERO_FORCE:              //零电流模式
            Shoot_Cmd->left_speed = 0;      //左摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            Shoot_Cmd->trigger_speed = 0;   //拨弹电机无速度
            break;
        case SHOOT_CLOSE:              //关闭摩擦轮模式
            Shoot_Cmd->left_speed = 0;      //左摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            Shoot_Cmd->trigger_speed = 0;   //拨弹电机无速度
            break;    
        case SHOOT_NORMAL:                  //摩擦轮正常工作模式
            Shoot_Cmd->left_speed = 6450;   //左摩擦轮速度
            Shoot_Cmd->right_speed = -6450;   //右摩擦轮速度
            Shoot_Cmd->trigger_speed = 0;   //拨弹电机速度
            break;
        case FIRE_MODE:		                //发射模式 
            Shoot_Cmd->left_speed = 6450;   //左摩擦轮速度
            Shoot_Cmd->right_speed = -6450;   //右摩擦轮速度
            Shoot_Cmd->trigger_speed = 2500;//(float)rc_ctrl.rc.ch[3]* 16.3636;   //拨弹电机速度
            break;
        default:
            Shoot_Cmd->left_speed = 0;      //左摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            Shoot_Cmd->trigger_speed = 0;   //拨弹电机无速度
            break;
    }
}

/**
 * @brief 设置发射机构电机目标速度
 * @param motors 发射机构电机实例数组指针
 * @param Chassis_Cmd 发射机构控制命令结构体指针
 * @details 将发射机构转速设置为电机的目标速度
 */
void Shoot_Motor_Set(MotorInstance  *motors, MotorInstance  *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd, VisionToGimbal *vision_feedback)
{
    if(motors == NULL ||  Shoot_Cmd == NULL || motors2 == NULL)
    {
        return;
    }
    // 将发射机构转速设置为电机的目标速度
    motors[0].motor_data->target_velocity = Shoot_Cmd->left_speed;
    motors[1].motor_data->target_velocity = Shoot_Cmd->right_speed;
    if(auto_aim_flag == 0)
    {
        motors2[0].motor_data->target_velocity = Shoot_Cmd->trigger_speed;
    }
    else
    {
        // if(vision_feedback->mode == 2)
        // {
            if(Shoot_Cmd->shoot_mode == FIRE_MODE)
            {
                Shoot_Cmd->trigger_speed = 2500;
            }
            else
            {
                Shoot_Cmd->trigger_speed = 0;
            }
            motors2[0].motor_data->target_velocity = Shoot_Cmd->trigger_speed;
        // }
        // else
        // {
        //     motors2[0].motor_data->target_velocity = 0;
        // }
    }
}

/**
 * @brief 清除发射机构电机PID控制器积分项
 * @param motors 发射机构电机实例数组指针
 * @details 在模式切换时调用，防止PID积分累积导致冲击
 */
void Shoot_PIDClear(MotorInstance  *motors)
{
    if(motors == NULL)
    {
        return;
    }
    //摩擦轮3508
    for(int i = 0;i < 2; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
}

/**
 * @brief 发射延迟测试
 * @param motors 发射机构摩擦轮电机实例数组指针
 * @param Shoot_Cmd 发射机构控制命令结构体指针
 * @return 最近一次测量的延迟时间，单位us；未测到有效结果时为0
 * @details 以遥控器切换到发射模式(FIRE_MODE)的时刻为起点，以摩擦轮被弹丸
 *          挤压产生掉速的时刻为终点，测得一次发射的总延迟。
 *          每次重新进入发射模式后开始新的一轮测量，需在控制循环中周期调用。
 */
float Fire_Latency_Test(MotorInstance *motors, Shoot_Ctrl_Cmd_s *Shoot_Cmd)
{
    static shoot_mode_e last_mode = SHOOT_ZERO_FORCE;   // 上一次的工作模式，用于检测模式切换
    static uint64_t fire_start_us = 0;                  // 进入发射模式的时刻
    static uint64_t armed_us = 0;                       // 摩擦轮加速到位的时刻
    static uint8_t  fired_armed = 0;                    // 摩擦轮已加速到位，等待掉速
    static uint8_t  fired_done = 0;                     // 本轮已检测到弹丸送出

    if(motors == NULL || Shoot_Cmd == NULL)
    {
        return fire_latency_us;
    }

    // 1.从其他模式切换到发射模式：记录起点，开始新一轮测量
    if(Shoot_Cmd->shoot_mode == FIRE_MODE && last_mode != FIRE_MODE)
    {
        fire_start_us = DWT_GetTimeline_us();
        fired_armed = 0;
        fired_done = 0;
        fire_latency_us = 0;
    }
    // 退出发射模式：复位，等待下一次切换
    else if(Shoot_Cmd->shoot_mode != FIRE_MODE)
    {
        fired_armed = 0;
        fired_done = 0;
    }
    last_mode = Shoot_Cmd->shoot_mode;

    // 2.通过摩擦轮反馈转速判断弹丸何时被送出
    if(Shoot_Cmd->shoot_mode == FIRE_MODE && !fired_done)
    {
        float left_speed  = fabsf(motors[0].motor_data->feedback->vel);
        float right_speed = fabsf(motors[1].motor_data->feedback->vel);
        float drop_line = FIRE_LATENCY_ARM_SPEED - FIRE_LATENCY_DROP_SPEED;

        if(!fired_armed)
        {
            // 摩擦轮已加速到额定转速，进入等待弹丸状态
            if(left_speed >= FIRE_LATENCY_ARM_SPEED && right_speed >= FIRE_LATENCY_ARM_SPEED)
            {
                fired_armed = 1;
                armed_us = DWT_GetTimeline_us();
            }
        }
        else if(left_speed < drop_line || right_speed < drop_line)
        {
            // 稳定期内不判定，避开加速超调造成的掉速
            if((float)(DWT_GetTimeline_us() - armed_us) >= FIRE_LATENCY_SETTLE_MS * 1000.0f)
            {
                // 弹丸挤压摩擦轮造成掉速，认为此刻弹丸被送出
                fire_latency_us = (float)(DWT_GetTimeline_us() - fire_start_us);
                fired_done = 1;
            }
        }
    }

    return fire_latency_us;
}

#endif
