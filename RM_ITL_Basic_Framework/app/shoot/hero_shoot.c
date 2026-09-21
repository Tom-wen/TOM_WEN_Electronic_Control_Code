#include "hero_shoot.h"
#ifdef COMPILE_HERO_SHOOT

//发射机构电机发送数据结构体实例
Shoot_Ctrl_Cmd_s shoot_cmd_send;
uint8_t Tri_flag = 0;

/**
 * @brief 发射机构主任务函数
 * @param argument 任务参数
 * @details 这是发射机构主任务函数，负责根据不同的模式控制发射机构
 */
void shoot_task(void *argument)
{
    static shoot_mode_e last_mode = SHOOT_ZERO_FORCE;
    Shoot_Init(Shoot_3508,Trigger_3508, &shoot_cmd_send);
    for(;;)
    {
        // 根据遥控器输入更新发射机构控制指令
        RemoteControlShoot(&shoot_cmd_send);
        Shoot_mode_update(&shoot_cmd_send);
        // 检测模式切换，清除PID积分器以避免突变
        if(last_mode == SHOOT_ZERO_FORCE && shoot_cmd_send.shoot_mode != SHOOT_ZERO_FORCE)
        {
            Shoot_PIDClear(Shoot_3508);
            Trigger_PIDClear(Trigger_3508);
        }
        last_mode = shoot_cmd_send.shoot_mode;
        switch (shoot_cmd_send.shoot_mode)
        {
            case SHOOT_ZERO_FORCE:
                // 零力模式：关闭所有电机输出
                Shoot_Motor_Status(Motor_Disable, Shoot_3508, 3);
                Shoot_Motor_Status(Motor_Disable, Trigger_3508, 1);
                break;
            case SHOOT_CLOSE:
                // 摩擦轮正常工作模式 
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 3);
                Shoot_Motor_Status(Motor_Enable, Trigger_3508, 1);                 
                Shoot_Motor_Set(Shoot_3508, &shoot_cmd_send, &vision_feedback);   
                break;
            case SHOOT_NORMAL:
                // 摩擦轮正常工作模式 
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 3);  
                Shoot_Motor_Status(Motor_Enable, Trigger_3508, 1);                
                Shoot_Motor_Set(Shoot_3508, &shoot_cmd_send, &vision_feedback);
                fire_trigger(&shoot_cmd_send, Trigger_3508);
                break;
            case FIRE_MODE:
                // 发射模式
                Shoot_Motor_Status(Motor_Enable, Shoot_3508, 3);  
                Shoot_Motor_Status(Motor_Enable, Trigger_3508, 1);              
                Shoot_Motor_Set(Shoot_3508, &shoot_cmd_send, &vision_feedback);
                fire_trigger(&shoot_cmd_send, Trigger_3508);
                break;                         
        }
        //上位机调PID用
        //PID_Test_Init(PID_K, Shoot_3508);
        //Shoot_3508[0].motor_data->target_velocity = rc_ctrl.rc.ch[1] * 5;
        // 更新底盘电机状态
        shoot_motor_updata(Shoot_3508, Trigger_3508);
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
void Shoot_Init(MotorInstance *motors,MotorInstance *motors2, Shoot_Ctrl_Cmd_s *Shoot_Cmd)
{   
    //发射机构3508单环PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 5.0f, 1.2f, 0.1f, 200.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 15.0f, 0.0f, 0.0f, 10.0f, 200.0f, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], 5.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 15.0f, 0.0f, 0.0f, 10.0f, 200.0f, 0.0f);

    PID_Init(&motors[2].motor_data->pid[single_loop], 5.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_inner], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_outer], 15.0f, 0.0f, 0.0f, 10.0f, 200.0f, 0.0f);

    PID_Init(&motors2[0].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 28.0f, 0.0f, 3.3f, 200.0f, 16000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.3f, 500.0f, 1000.0f, 0.0f);

    //PID清零
    Shoot_PIDClear(Shoot_3508);
   
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
    static uint16_t last_key_state = 0;
    if (KEY_F && !(last_key_state & (1 << 5)))  // F键对应bit5
    {
        Shoot_Cmd->shoot_mode = SHOOT_NORMAL;  // 设置为正常模式
        last_key_state = rc_ctrl.key;
        return;  // 提前返回，不执行后面的模式判断
    }
    if (KEY_G && !(last_key_state & (1 << 10)))  // G键对应bit5
    {
        Shoot_Cmd->shoot_mode = SHOOT_CLOSE;  // 设置为正常模式
        last_key_state = rc_ctrl.key;
        return;  // 提前返回，不执行后面的模式判断
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
    #ifdef FS_REMOTE
    // 根据左拨杆位置设置云台模式
    switch (left_switch2)
    {
        case switch_up:
            if(right_switch2 == switch_up)
            {
                Shoot_Cmd->shoot_mode = SHOOT_CLOSE;
                break;    
            }   
            else if(right_switch2 == switch_down)
            {
                Shoot_Cmd->shoot_mode = SHOOT_CLOSE;
                break;
            }         
            break;            
        case switch_down:
            if(right_switch2 == switch_up)
            {
                Shoot_Cmd->shoot_mode = SHOOT_NORMAL;
                break;    
            }
            else if(right_switch2 == switch_down)
            {
                Shoot_Cmd->shoot_mode = FIRE_MODE;
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
void shoot_motor_updata(MotorInstance *motors,MotorInstance *motors2)
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
            Shoot_Cmd->mid_speed = 0;       //中摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            break;
        case SHOOT_CLOSE:              //摩擦轮关闭模式
            Shoot_Cmd->left_speed = 0;      //左摩擦轮无速度
            Shoot_Cmd->mid_speed = 0;       //中摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            break;            
        case SHOOT_NORMAL:                  //摩擦轮正常工作模式
            Shoot_Cmd->left_speed = 2900;   //左摩擦轮速度
            Shoot_Cmd->mid_speed = 2900;    //中摩擦轮速度
            Shoot_Cmd->right_speed = 2900;   //右摩擦轮速度
            break;
        case FIRE_MODE:		                //发射模式 
            Shoot_Cmd->left_speed = 2900;   //左摩擦轮速度
            Shoot_Cmd->mid_speed = 2900;    //中摩擦轮速度
            Shoot_Cmd->right_speed = 2900;   //右摩擦轮速度
            break;
        default:
            Shoot_Cmd->left_speed = 0;      //左摩擦轮无速度
            Shoot_Cmd->mid_speed = 0;       //中摩擦轮无速度
            Shoot_Cmd->right_speed = 0;     //右摩擦轮无速度
            break;
    }
}

/**
 * @brief 设置发射机构电机目标速度
 * @param motors 发射机构电机实例数组指针
 * @param Chassis_Cmd 发射机构控制命令结构体指针
 * @details 将发射机构转速设置为电机的目标速度
 */
void Shoot_Motor_Set(MotorInstance  *motors, Shoot_Ctrl_Cmd_s *Shoot_Cmd, VisionToGimbal *vision_feedback)
{
    if(motors == NULL ||  Shoot_Cmd == NULL)
    {
        return;
    }
    // 将发射机构转速设置为电机的目标速度
    motors[0].motor_data->target_velocity = Shoot_Cmd->left_speed;
    motors[1].motor_data->target_velocity = Shoot_Cmd->mid_speed;
    motors[2].motor_data->target_velocity = Shoot_Cmd->right_speed;
    
}

void fire_trigger(Shoot_Ctrl_Cmd_s *Shoot_Cmd, MotorInstance *motors)
{
    static uint8_t leg_zero_two = 0;
    static uint8_t last_mouse_left = 0;
    static uint16_t last_key_state = 0;
    static uint8_t last_mode = 0;
    // static uint8_t Tri_flag = 0;
    // static uint32_t last_fire_tick  = 0;   // 上次射击时刻（ms）

    if (!leg_zero_two)
    {
        Shoot_Cmd->trigger_angle = motors[0].motor_data->feedback->pos / 22.75278f;
        leg_zero_two = 1;
    }
    if (rc_ctrl.key & (1 << 9)) // KEY_F
    {
        Tri_flag = 1;
    }
    else if (rc_ctrl.key & (1 << 10)) // KEY_G
    {
        Tri_flag = 0;
    }
    
    // 检测 SHOOT_NORMAL -> FIRE_MODE 切换，只执行一次 trigger_angle 累加
    if(last_mode == SHOOT_NORMAL && shoot_cmd_send.shoot_mode == FIRE_MODE)
    {
        shoot_cmd_send.trigger_angle -= 60 * 3591 / 187;
    }
       
    if (Tri_flag == 1)
    {
        //if ((received_data.rc_ctrl.key & (1 << 14)) && !(last_key_state & (1 << 14)))  // KEY_V 上升沿
        if (rc_ctrl.mouse_left && !last_mouse_left)
        {
            Shoot_Cmd->trigger_angle -= 60 * 3591 / 187;
        }
    }
    last_mouse_left = rc_ctrl.mouse_left;
    last_mode = shoot_cmd_send.shoot_mode;
    last_key_state = rc_ctrl.key;
    motors[0].motor_data->target_position = Shoot_Cmd->trigger_angle;
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
    for(int i = 0;i < 3; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
}

void Trigger_PIDClear(MotorInstance  *motors)
{
    if(motors == NULL)
    {
        return;
    }
    //拨弹盘3508
    for(int i = 0;i < 1; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
}

#endif 
