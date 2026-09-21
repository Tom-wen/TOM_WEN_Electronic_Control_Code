#include "agv_chassis.h"
#include "data_processing.h"

#ifdef COMPILE_AGV_CHASSIS

//底盘电机发送数据
Chassis_Ctrl_Cmd_s chassis_cmd_send;
// 挡位控制
static float gear_limits[] = {200.0f, 400.0f, 600.0f};  // 各挡位速度限制

/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 */
void chassis_task(void *argument)
{
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;
    Chassis_Init(Chassis_3508, Chassis_6020);
    for(;;)
    {
        RemoteControlChassis(&chassis_cmd_send);
        Chassis_mode_update(&chassis_cmd_send);
        // 检测模式切换，清除PID积分器
        if(last_mode == CHASSIS_ZERO_FORCE && chassis_cmd_send.chassis_mode != CHASSIS_ZERO_FORCE)
        {
            Chassis_PIDClear(Chassis_3508, Chassis_6020);
        }
        last_mode = chassis_cmd_send.chassis_mode;
        switch (chassis_cmd_send.chassis_mode)
        {
            case CHASSIS_ZERO_FORCE:
                Chassis_Motor_Status(Motor_Disable, Chassis_3508, Chassis_6020, 4);
                break;
            case CHASSIS_NO_FOLLOW:
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, Chassis_6020, 4);
                AGV_Cal(&chassis_cmd_send);
                AGV_Motor_Set(Chassis_3508, Chassis_6020, &chassis_cmd_send);
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, Chassis_6020, 4);
                AGV_Cal(&chassis_cmd_send);
                AGV_Motor_Set(Chassis_3508, Chassis_6020, &chassis_cmd_send);
                break;
            case CHASSIS_ROTATE:
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, Chassis_6020, 4);
                AGV_Cal(&chassis_cmd_send);
                AGV_Motor_Set(Chassis_3508, Chassis_6020, &chassis_cmd_send);
                break;                            
        }
        chassis_motor_updata(Chassis_3508, Chassis_6020);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 底盘初始化函数
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 */
void Chassis_Init(MotorInstance *motors1, MotorInstance *motors2)
{   
    //底盘3508PID初始化
    PID_Init(&motors1[0].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors1[0].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors1[0].motor_data->pid[cascade_outer], 25.0f, 0.0f, 4.0f, 0.0f, 50.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_Init(&motors1[1].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors1[1].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors1[1].motor_data->pid[cascade_outer], 25.0f, 0.0f, 4.0f, 0.0f, 50.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors1[2].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors1[2].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors1[2].motor_data->pid[cascade_outer], 25.0f, 0.0f, 4.0f, 0.0f, 50.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors1[3].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors1[3].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors1[3].motor_data->pid[cascade_outer], 25.0f, 0.0f, 4.0f, 0.0f, 50.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    //底盘6020PID初始化
    PID_Init(&motors2[0].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 120.0f, 80.0f, 0.0f, 0.0f, 4000.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 1.4f, 0.0f, 0.0f, 0.0f, 50.0f, 70.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_Init(&motors2[1].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[1].motor_data->pid[cascade_inner], 120.0f, 80.0f, 0.0f, 0.0f, 4000.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors2[1].motor_data->pid[cascade_outer], 1.4f, 0.0f, 0.0f, 0.0f, 50.0f, 70.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors2[2].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[2].motor_data->pid[cascade_inner], 120.0f, 80.0f, 0.0f, 0.0f, 4000.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors2[2].motor_data->pid[cascade_outer], 1.4f, 0.0f, 0.0f, 0.0f, 50.0f, 70.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors2[3].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[3].motor_data->pid[cascade_inner], 120.0f, 80.0f, 0.0f, 0.0f, 4000.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors2[3].motor_data->pid[cascade_outer], 1.4f, 0.0f, 0.0f, 0.0f, 50.0f, 70.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    //底盘6020初始回零
    motors2[0].motor_data->target_position = Chassis_6020_offset_angle1;
    motors2[1].motor_data->target_position = Chassis_6020_offset_angle2;
    motors2[2].motor_data->target_position = Chassis_6020_offset_angle3;
    motors2[3].motor_data->target_position = Chassis_6020_offset_angle4;
    //初始化底盘速度挡位
    chassis_cmd_send.gear_level = 1;    
    chassis_motor_updata(motors1, motors2);
}

/**
 * @brief 设置底盘电机状态
 * @param status 电机状态（启用/禁用）
 * @param motors 3508电机实例指针
 * @param motors1 6020电机实例指针
 * @param motor_count 电机数量
 */
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors,  MotorInstance *motors1, uint8_t motor_count)
{
    switch (status)
    {
    case Motor_Enable:
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_enable(motors[i].motor_data);
            DJI_Motor_enable(motors1[i].motor_data);
        }
        break;
    case Motor_Disable:
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
            DJI_Motor_disable(motors1[i].motor_data);
        }
        break;
    default:
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
            DJI_Motor_disable(motors1[i].motor_data);
        }
        break;
    }
}

/**
 * @brief 更新底盘模式
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    #ifdef chassis_board
        if(received_data.sbus_online == 0)
        {
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            return;
        }
    #else
        if(sbus_online == 0)
        {
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            return;
        }
    #endif
 
    #ifdef DJI_REMOTE
        switch (left_switch)
        {
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_NO_FOLLOW;
            break; 
        case switch_mid:
            if(right_switch == switch_down)
            {
                Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            else
            {
                Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            }
            break;
        case switch_up:
            Chassis_Cmd->chassis_mode = CHASSIS_ROTATE;
            break;    
        default:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break;
        }
    #endif
    #ifdef FS_REMOTE
        switch (right_switch)
        {
        case switch_up:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break; 
        case switch_mid:
            Chassis_Cmd->chassis_mode = CHASSIS_NO_FOLLOW;
            break;
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;    
        default:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break;
        }
    #endif
#ifdef chassis_board
    static uint8_t last_switch_state = 0xFF;
    static chassis_mode_e current_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    Chassis_Cmd->chassis_mode = current_mode;

    // 开关切换模式（边沿检测，只有开关位置变化时才切换）
    if ((received_left_switch != last_switch_state) && (received_left_switch != switch_down))
    {
        switch (received_left_switch)
        {
            case switch_mid:
                current_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                break;
            case switch_up:
                current_mode = CHASSIS_ROTATE;
                break;
            default:
                current_mode = CHASSIS_ZERO_FORCE;
                break;
        }
        Chassis_Cmd->chassis_mode = current_mode;
        last_switch_state = received_left_switch;
    }
#endif
 
}

/**
 * @brief 更新底盘电机控制数据
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 */
void chassis_motor_updata(MotorInstance *motors1, MotorInstance *motors2)
{
    if(motors1 == NULL || motors2==NULL)
    {
        return;
    }
    //底盘3508电机给电流
    motors1->motor_control(motors1);
    //底盘6020电机给电流
    motors2->motor_control(motors2);
}

/**
 * @brief 遥控器控制底盘
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd) 
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }

    static float vx = 0.0f;
    static float vy = 0.0f;

    // 处理挡位切换
    Gear_Switch_Handler();
    // 获取当前挡位的速度限制
    float current_limit = Get_Current_Gear_Limit();
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch(Chassis_Cmd->chassis_mode) 
    {
        case CHASSIS_FOLLOW_GIMBAL_YAW:                         //跟随云台
            Chassis_Cmd->vx = (float)received_data.rc_ctrl.rc.ch[1];
            Chassis_Cmd->vy = (float)received_data.rc_ctrl.rc.ch[0];
            Chassis_Cmd->w = -(float)received_data.rc_ctrl.rc.ch[4]*0.01;
            break;
        case CHASSIS_NO_FOLLOW:                                 //不跟随云台
            Chassis_Cmd->vx = (float)received_data.rc_ctrl.rc.ch[1];
            Chassis_Cmd->vy = (float)received_data.rc_ctrl.rc.ch[0];
            Chassis_Cmd->w = -(float)received_data.rc_ctrl.rc.ch[4]*0.01;
            break;
        case CHASSIS_ROTATE:		                            //小陀螺模式
            Chassis_Cmd->vx = (float)received_data.rc_ctrl.rc.ch[1];
            Chassis_Cmd->vy = (float)received_data.rc_ctrl.rc.ch[0];
            Chassis_Cmd->w = -(float)received_data.rc_ctrl.rc.ch[4]*0.01;
            break;
	    case CHASSIS_ZERO_FORCE:		                        //零电流模式
            Chassis_Cmd->vx = 0;
            Chassis_Cmd->vy = 0;
            Chassis_Cmd->w = 0;
            break;
        default:
            break;
    }
}

/**
 * @brief AGV运动学计算
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void AGV_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    float vw = Chassis_Cmd->w * Radius;
    //3508转速
    Chassis_Cmd->wheel3508_rpm[0] = sqrt(pow((Chassis_Cmd->vy - vw * cos45), 2) + pow((Chassis_Cmd->vx - vw * sin45), 2));
    Chassis_Cmd->wheel3508_rpm[1] = sqrt(pow((Chassis_Cmd->vy - vw * cos45), 2) + pow((Chassis_Cmd->vx + vw * sin45), 2));
    Chassis_Cmd->wheel3508_rpm[2] = sqrt(pow((Chassis_Cmd->vy + vw * cos45), 2) + pow((Chassis_Cmd->vx - vw * sin45), 2));
    Chassis_Cmd->wheel3508_rpm[3] = -sqrt(pow((Chassis_Cmd->vy + vw * cos45), 2) + pow((Chassis_Cmd->vx + vw * sin45), 2));

    //6020要转角度
    Chassis_Cmd->wheel6020_relative_angle[0] = atan2((Chassis_Cmd->vy - vw * cos45), (Chassis_Cmd->vx - vw * sin45)) * angle_change;
    Chassis_Cmd->wheel6020_relative_angle[1] = atan2((Chassis_Cmd->vy - vw * cos45), (Chassis_Cmd->vx + vw * sin45)) * angle_change;
    Chassis_Cmd->wheel6020_relative_angle[2] = atan2((Chassis_Cmd->vy + vw * cos45), (Chassis_Cmd->vx + vw * sin45)) * angle_change;
    Chassis_Cmd->wheel6020_relative_angle[3] = atan2((Chassis_Cmd->vy + vw * cos45), (Chassis_Cmd->vx - vw * sin45)) * angle_change;

    Chassis_Cmd->wheel6020_target_angle[0] = Chassis_6020_offset_angle1 + Chassis_Cmd->wheel6020_relative_angle[0];
    Chassis_Cmd->wheel6020_target_angle[1] = Chassis_6020_offset_angle2 + Chassis_Cmd->wheel6020_relative_angle[1];
    Chassis_Cmd->wheel6020_target_angle[2] = Chassis_6020_offset_angle3 + Chassis_Cmd->wheel6020_relative_angle[2];
    Chassis_Cmd->wheel6020_target_angle[3] = Chassis_6020_offset_angle4 + Chassis_Cmd->wheel6020_relative_angle[3];

}

/**
 * @brief 设置AGV电机目标值
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void AGV_Motor_Set(MotorInstance  *motors1, MotorInstance  *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(motors1 == NULL || motors2 == NULL ||  Chassis_Cmd == NULL)
    {
        return;
    }
    for(int i = 0; i < 4; i++)
    {
        motors1[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i] * 3;
        motors2[i].motor_data->target_position = Chassis_Cmd->wheel6020_target_angle[i];
    }
}

/**
 * @brief 清除底盘PID控制器积分项
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 */
void Chassis_PIDClear(MotorInstance  *motors1, MotorInstance  *motors2)
{

}

/**
 * @brief 处理ctrl键挡位切换
 */
void Gear_Switch_Handler(void)
{
    static uint16_t last_key_state = 0;  // 按键边沿检测
    // ctrl键边沿检测
    if (RECEIVE_KEY_CTRL && !(last_key_state & (1 << 5)))
    {
        // 循环切换挡位
        chassis_cmd_send.gear_level = (chassis_cmd_send.gear_level % 3) + 1;
    }
    last_key_state = received_data.rc_ctrl.key;
}

/**
 * @brief 获取当前挡位的速度限制
 * @retval 当前挡位的最大速度
 */
float Get_Current_Gear_Limit(void)
{
    return gear_limits[chassis_cmd_send.gear_level - 1];  // 数组从0开始，挡位从1开始
}

void KeyAccumulator_Dual_Instant_receive(uint16_t key_inc, uint16_t key_dec,
                               float *value, float step, float limit)
{
    if (received_data.rc_ctrl.key & key_inc)
    {
        *value += step;
        if (*value > limit) *value = limit;
    }
    else if (received_data.rc_ctrl.key & key_dec)
    {
        *value -= step;
        if (*value < -limit) *value = -limit;
    }
    else
    {
        *value = 0;  // 直接归零，无衰减
    }
}

#endif
