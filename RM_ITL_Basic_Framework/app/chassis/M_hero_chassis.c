#include "M_hero_chassis.h"
#include "data_processing.h"
 #ifdef COMPILE_M_POWER
  #include "Master_power.h"
  #endif
#ifdef COMPILE_M_HERO_CHASSIS
//底盘电机发送数据结构体实例
Chassis_Ctrl_Cmd_s chassis_cmd_send;

// 挡位控制
static float gear_limits[] = {200.0f, 400.0f, 600.0f};  // 各挡位速度限制
 
/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 * @details 这是底盘控制的主任务函数，负责根据不同的模式控制底盘运动
 */
void chassis_task(void *argument)
{
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;
    Chassis_Init(Chassis_3508, &chassis_cmd_send);
    for(;;)
    {
        // 根据遥控器输入更新底盘控制指令
        RemoteControlChassis(&chassis_cmd_send);
        Chassis_mode_update(&chassis_cmd_send);
        // 检测模式切换，清除PID积分器以避免突变
        if(last_mode == CHASSIS_ZERO_FORCE && chassis_cmd_send.chassis_mode != CHASSIS_ZERO_FORCE)
        {
            Chassis_PIDClear(Chassis_3508);
        }
        last_mode = chassis_cmd_send.chassis_mode;
        switch (chassis_cmd_send.chassis_mode)
        {
            case CHASSIS_ZERO_FORCE:
                // 零力模式：关闭所有电机输出
                Chassis_Motor_Status(Motor_Disable, Chassis_3508, 4);
                break;
            case CHASSIS_NO_FOLLOW:
                // 无跟随模式：底盘独立运动，不跟随云台
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, 4);
                Mecanum_Calc(&chassis_cmd_send);
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                // 跟随云台偏航角模式：底盘运动时会自动补偿云台偏航角
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, 4);
                Absolute_Cal(&chassis_cmd_send, 0); 
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;
            case CHASSIS_ROTATE:
                // 自旋模式：底盘原地旋转
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, 4);
                Absolute_Cal(&chassis_cmd_send, Chassis_relative_angle(Gimbal_4310));
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;                           
        }
        //上位机调PID用
        //PID_Test_Init(PID_K, Chassis_3508);
        #ifdef COMPILE_M_POWER
        MasterPower_Update((left_switch == switch_up) ? 130.0f : 30.0f);
        #endif
        // 更新底盘电机状态
        chassis_motor_updata(Chassis_3508);
        // PID 计算完后，再用实际数据做功率控制（下一帧生效）
        #ifdef COMPILE_POWER
        power_data_sent(Chassis_3508, chassis_3508_motor, 4);
        #endif
        // 控制任务执行频率（2ms周期）
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 底盘初始化函数
 * @param motors 底盘电机实例数组指针
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 初始化底盘各电机的PID控制器参数
 */
void Chassis_Init(MotorInstance *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{   
    //底盘3508单环PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 15.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], 15.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[2].motor_data->pid[single_loop], 15.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[3].motor_data->pid[single_loop], 15.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    //底盘跟随云台旋转PID控制器初始化
    PID_Init(&Chassis_Cmd->cascade_pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);//未使用
    PID_Init(&Chassis_Cmd->cascade_pid[cascade_inner], 8.0f, 0.0f, 0.0f, 20.0f, 500.0f, 0.0f);
    PID_Init(&Chassis_Cmd->cascade_pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, 0.0f);
    //PID清零
    Chassis_PIDClear(Chassis_3508);
    //初始化底盘速度挡位
    Chassis_Cmd->gear_level = 1;
    
  #ifdef COMPILE_M_POWER
      Motor_feedback *motor_fbs[4];
      for (int i = 0; i < 4; i++)
          motor_fbs[i] = motors[i].motor_data->feedback;
      MasterPower_Init(motor_fbs, 0.05f, 1.2f, 2.78f);
  #endif
}

/**
 * @brief 设置底盘电机状态
 * @param status 电机状态（启用或禁用）
 * @param motors 底盘电机实例数组指针
 * @param motor_count 电机数量
 */
void Chassis_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
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
 * @brief 更新底盘工作模式
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 根据遥控器开关状态更新底盘工作模式
 */
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    // 如果SBUS信号断开，则强制进入零力模式
    if(sbus_online == 0)
    {
        Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }
    #ifdef DJI_REMOTE
    // 根据遥控器左侧开关位置选择底盘模式
    switch (left_switch)
    {
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_NO_FOLLOW;
            break; 
        case switch_mid:
            Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
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
            Chassis_Cmd->chassis_mode = CHASSIS_NO_FOLLOW;
            break; 
        case switch_mid:
            Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_ROTATE;
            break;    
        default:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break;
    }
    #endif
    #ifdef VT_03_REMOTE
        static uint8_t last_switch_state = 0xFF;
        static uint16_t last_key_state_mode = 0;  // 按键边沿检测（模式切换用）
        static chassis_mode_e current_mode = CHASSIS_FOLLOW_GIMBAL_YAW;  // 用 static 保存模式

        Chassis_Cmd->chassis_mode = current_mode;  // 使用保存的模式

        // R键循环切换模式（手动边沿检测）
        if (KEY_R && !(last_key_state_mode & (1 << 8)))
        {
            if (current_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
            {
                current_mode = CHASSIS_ROTATE;
            }
            else if (current_mode == CHASSIS_ROTATE)
            {
                current_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            Chassis_Cmd->chassis_mode = current_mode;
        }
        // 开关切换模式（边沿检测，只有开关位置变化时才切换）
        else if ((left_switch != last_switch_state) && (left_switch != switch_down))
        {
            switch (left_switch)
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
            last_switch_state = left_switch;
        }
        // 更新按键状态（每次循环末尾）
        last_key_state_mode = rc_ctrl.key;
    #endif


}

/**
 * @brief 更新底盘电机控制
 * @param motors 底盘电机实例数组指针
 * @details 执行电机控制算法并更新电机输出
 */
void chassis_motor_updata(MotorInstance *motors)
{
    if(motors == NULL)
    {
        return;
    }
    //更新底盘3508电机控制输出
    motors->motor_control(motors);
}
/**
 * @brief 根据遥控器输入控制底盘运动
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 解析遥控器通道数据，计算底盘在不同模式下的运动指令
 */
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd) 
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    /***********************************确定底盘四个电机的目标速度*****************************************/
    // 设置底盘跟随云台旋转的速度
    Set_follow_wz(Gimbal_4310, Chassis_Cmd);
    static float vx = 0.0f;
    static float vy = 0.0f;

    // 处理挡位切换
    Gear_Switch_Handler();
    // 获取当前挡位的速度限制
    float current_limit = Get_Current_Gear_Limit();

    switch(Chassis_Cmd->chassis_mode) 
    {
        case CHASSIS_FOLLOW_GIMBAL_YAW:     //跟随云台模式
            // W/S控制前后，D/A控制左右（使用当前挡位限速）
            KeyAccumulator_Dual_Instant(KEY_W, KEY_S, &vx, 3.0f, current_limit);
            KeyAccumulator_Dual_Instant(KEY_D, KEY_A, &vy, 3.0f, current_limit);

            // 键盘有操作时用键盘，否则用摇杆
            if (rc_ctrl.key != 0)
            {
                Chassis_Cmd->vx = vx;
                Chassis_Cmd->vy = -vy;
            }
            else
            {
                Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1];
                Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0];    
            }
            Chassis_Cmd->w = -(Chassis_Cmd->cascade_pid[cascade_inner].Out * 5); // 使用PID输出作为角速度
            break;
        case CHASSIS_NO_FOLLOW:             //不跟随云台模式
            // W/S控制前后，D/A控制左右（使用当前挡位限速）
            KeyAccumulator_Dual_Instant(KEY_W, KEY_S, &vx, 3.0f, current_limit);
            KeyAccumulator_Dual_Instant(KEY_D, KEY_A, &vy, 3.0f, current_limit);

            // 键盘有操作时用键盘，否则用摇杆
            if (rc_ctrl.key != 0)
            {
                Chassis_Cmd->vx = vx;
                Chassis_Cmd->vy = -vy;
            }
            else
            {
                Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1];
                Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0];    
            }
            Chassis_Cmd->w = 0;
            break;
        case CHASSIS_ROTATE:		        //小陀螺模式（自旋）
            // W/S控制前后，D/A控制左右（使用当前挡位限速）
            KeyAccumulator_Dual_Instant(KEY_W, KEY_S, &vx, 3.0f, current_limit);
            KeyAccumulator_Dual_Instant(KEY_D, KEY_A, &vy, 3.0f, current_limit);

            // 键盘有操作时用键盘，否则用摇杆
            if (rc_ctrl.key != 0)
            {
                Chassis_Cmd->vx = vx;
                Chassis_Cmd->vy = -vy;
            }
            else
            {
                Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1];
                Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0];    
            }
            if(chassis_cmd_send.gear_level ==1){
                Chassis_Cmd->w =  3000;
            }
            else if(chassis_cmd_send.gear_level ==2){
                Chassis_Cmd->w =  3800;
            }
            else if(chassis_cmd_send.gear_level ==3){
                Chassis_Cmd->w =  4200;
            }
            break;
	    case CHASSIS_ZERO_FORCE:		    //零电流模式
            Chassis_Cmd->vx = 0;            // 无前后移动
            Chassis_Cmd->vy = 0;            // 无左右移动
            Chassis_Cmd->w = 0;             // 无旋转
            break;
        default:
            break;
    }
}

/**
 * @brief 清除底盘PID控制器积分项
 * @param motors 底盘电机实例数组指针
 * @details 在模式切换时调用，防止PID积分累积导致冲击
 */
void Chassis_PIDClear(MotorInstance  *motors)
{
    if(motors == NULL)
    {
        return;
    }
    //底盘3508
    for(int i = 0;i < 4; i++)
    {
        PID_Clear(motors[i].motor_data->pid);
    }
    PID_Clear(chassis_cmd_send.cascade_pid);
}

/**
  * @brief  将云台坐标系的速度转换为底盘坐标系的速度
  * @param  Chassis_Cmd 底盘控制命令结构体指针
  * @param  angle 云台相对于底盘的角度（单位：度）
  * @retval 无
  * @attention 输入的速度应在云台坐标系下
  */
void Absolute_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd, float angle)
{
    float angle_hd = angle;              // 弧度
    Chassis_Ctrl_Cmd_s temp_speed;                  // 临时速度变量
    temp_speed.w = Chassis_Cmd->w;              // 放大角速度
    // 坐标变换：将云台坐标系的速度分量转换到底盘坐标系
    temp_speed.vx = Chassis_Cmd->vx * cos(angle_hd) - Chassis_Cmd->vy * sin(angle_hd);
    temp_speed.vy = Chassis_Cmd->vx * sin(angle_hd) + Chassis_Cmd->vy * cos(angle_hd);
    Mecanum_Calc(&temp_speed);                      // 计算麦轮转速
    // 将计算结果复制回原结构体
    for(int i = 0; i < 4; i++)
    {
        Chassis_Cmd->wheel3508_rpm[i] = temp_speed.wheel3508_rpm[i];
    }
}

/**
 * @brief 麦轮底盘运动学解算
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 根据底盘坐标系下的速度指令计算四个麦轮的目标转速
 */
void Mecanum_Calc(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    Chassis_Cmd->wheel3508_rpm[0] = (Chassis_Cmd->vx - Chassis_Cmd->vy - (LENGTH_A + LENGTH_B) * MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w);
    Chassis_Cmd->wheel3508_rpm[1] = (-Chassis_Cmd->vx - Chassis_Cmd->vy - (LENGTH_A + LENGTH_B) * MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w);
    Chassis_Cmd->wheel3508_rpm[2] = (-Chassis_Cmd->vx + Chassis_Cmd->vy - (LENGTH_A + LENGTH_B) * MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w);
    Chassis_Cmd->wheel3508_rpm[3] = (Chassis_Cmd->vx + Chassis_Cmd->vy - (LENGTH_A + LENGTH_B) * MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w);
}

/**
 * @brief 设置麦轮电机目标速度
 * @param motors 底盘电机实例数组指针
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 将计算出的轮子转速设置为电机的目标速度
 */
void Mecanum_Motor_Set(MotorInstance  *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(motors == NULL ||  Chassis_Cmd == NULL)
    {
        return;
    }
    // 将计算出的轮子转速设置为各电机的目标速度（乘以2.5可能是减速比或其他转换系数）
    for(int i = 0; i < 4; i++)
    {     
       // motors[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i] * 5;
        #ifdef COMPILE_POWER
                motors[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i] * 5; //*Chassis_3508_Get_Limited_Number(i);
            #else
               motors[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i] * 5;
            #endif
    }
}

/**
 * @brief 计算底盘跟随云台旋转所需角速度
 * @param motors 云台电机实例数组指针
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 通过PID控制器使底盘跟随云台旋转，保持相对角度稳定
 */
void Set_follow_wz(MotorInstance  *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    const float max_angle = 360;                            // 最大角度值（一圈360度）   
    float current = motors[0].motor_data->feedback->pos * 57.296f; // 将编码器值转为角度值(0~360度)
    float target = yaw_offset;                              // 目标角度为偏航角偏移量
    // 处理角度跨越问题，确保目标角度与当前角度在同一连续区间内
    while (target - current > max_angle / 2) 
    {
        target -= max_angle;
    } 
    while (target - current < -max_angle / 2) 
    {
        target += max_angle;
    }
    // 使用串级PID控制器计算跟随所需的角速度
    PID_CascadeCalc(Chassis_Cmd->cascade_pid, target, current, (motors[0].motor_data->feedback->vel * rads));
}

float Chassis_relative_angle(MotorInstance  *motors)
{
    float SpinTop_Angle = 0;
    SpinTop_Angle = motors[0].motor_data->feedback->pos - yaw_encoder_offset; // 直接弧度相减
    if (SpinTop_Angle > 6.28318f)
    {
        SpinTop_Angle -= 6.28318f;
    }
    if (SpinTop_Angle < 0)
    {
        SpinTop_Angle += 6.28318f;
    }
    return SpinTop_Angle;
}

/**
 * @brief 处理ctrl键挡位切换
 */
void Gear_Switch_Handler(void)
{
    static uint16_t last_key_state = 0;  // 按键边沿检测
    // ctrl键边沿检测
    if (KEY_CTRL && !(last_key_state & (1 << 5)))
    {
        // 循环切换挡位
        chassis_cmd_send.gear_level = (chassis_cmd_send.gear_level % 3) + 1;
    }
    last_key_state = rc_ctrl.key;
}

/**
 * @brief 获取当前挡位的速度限制
 * @retval 当前挡位的最大速度
 */
float Get_Current_Gear_Limit(void)
{
    return gear_limits[chassis_cmd_send.gear_level - 1];  // 数组从0开始，挡位从1开始
}

#ifdef COMPILE_POWER
//给功率控制传递电机转速和电流
void power_data_sent(MotorInstance *motors, MotorPower* power_motor, uint8_t motor_count)
{
    for(int i = 0; i < motor_count; i++)
    {
        float decay = Chassis_3508_Get_Limited_Number(i);
        power_motor[i].current_speed = motors[i].motor_data->feedback->vel;
        power_motor[i].current_current = (motors[i].motor_data->pid[cascade_inner].Out);
        power_motor[i].target_speed = motors[i].motor_data->target_velocity;
    }
}
#endif
#endif
