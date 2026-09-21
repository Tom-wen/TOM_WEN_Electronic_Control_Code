#include "robotic_swing_chassis.h"
#include "control_power.h"
#include "user_lib.h"
#ifdef ROBOTIC_SWING_CHASSIS

//底盘电机发送数据结构体实例
Chassis_Ctrl_Cmd_s chassis_cmd_send;
uint8_t chassis_online = 0;

 
/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 * @details 这是底盘控制的主任务函数，负责根据不同的模式控制底盘运动
 */
void chassis_task(void *argument)
{
    Chassis_Init(Chassis_3508, &chassis_cmd_send);
    for(;;)
    {
        // 根据遥控器输入更新底盘控制指令
        RemoteControlChassis(&chassis_cmd_send);
        Chassis_mode_update(&chassis_cmd_send);
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
                Mecanum_Calc(&chassis_cmd_send);
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;
            case CHASSIS_ROTATE:
                // 自旋模式：底盘原地旋转
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, 4);
                Absolute_Cal(&chassis_cmd_send, Chassis_relative_angle(Gimbal_6020));
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;
            case CHASSIS_DEAD: 
                Chassis_Motor_Status(Motor_Enable, Chassis_3508, 4);
                Absolute_Cal(&chassis_cmd_send, Chassis_relative_angle(Gimbal_6020));
                Mecanum_Motor_Set(Chassis_3508, &chassis_cmd_send);
                break;                            
        }
        #ifdef COMPILE_M_POWER
        MasterPower_Update((chassis_cmd_send.chassis_mode == CHASSIS_ROTATE) ? 90.0f : 90.0f);
        #endif
        //chassis_motor_updata(Chassis_3508);
        Check_Chassis_Offline();//检查底盘是否离线
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
    PID_Init(&motors[0].motor_data->pid[single_loop], 25.0f, 0.01f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], 25.0f, 0.01f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[2].motor_data->pid[single_loop], 25.0f, 0.01f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[3].motor_data->pid[single_loop], 25.0f, 0.01f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    //底盘跟随云台旋转PID控制器初始化
    PID_Init(&Chassis_Cmd->cascade_pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);//未使用
    PID_Init(&Chassis_Cmd->cascade_pid[cascade_inner], 10.0f, 0.0f, 0.0f, 20.0f, 1000.0f, 0.0f);
    PID_Init(&Chassis_Cmd->cascade_pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 500.0f, 0.0f);
    //PID清零
    Chassis_PIDClear(Chassis_3508);
     #ifdef COMPILE_M_POWER
      Motor_feedback *motor_fbs[4];
      for (int i = 0; i < 4; i++)
          motor_fbs[i] = motors[i].motor_data->feedback;
      MasterPower_Init(motor_fbs, 0.2f, 2.3f, 8.4f);
      MasterPower_SetRLSEnable(0);
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
    Set_follow_wz(Gimbal_6020, Chassis_Cmd);
    static float vx = 0.0f;
    static float vy = 0.0f;

    switch(Chassis_Cmd->chassis_mode) 
    {
        case CHASSIS_FOLLOW_GIMBAL_YAW:     //跟随云台模式
            Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1]*sensitivity;
            Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0]*sensitivity;    
            Chassis_Cmd->w = -(Chassis_Cmd->cascade_pid[cascade_inner].Out *0.005); // 使用PID输出作为角速度
            break;
        case CHASSIS_NO_FOLLOW:             //不跟随云台模式
            Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1]*sensitivity;
            Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0]*sensitivity;    
            Chassis_Cmd->w = 0;
            break;
        case CHASSIS_ROTATE:		        //小陀螺模式（自旋）
            Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1]*sensitivity;
            Chassis_Cmd->vy = -(float)rc_ctrl.rc.ch[0]*sensitivity;   
            Chassis_Cmd->w = CHASSIS_WZ_SET; 
            break;
	    case CHASSIS_ZERO_FORCE:		    //零电流模式
            Chassis_Cmd->vx = 0;            // 无前后移动
            Chassis_Cmd->vy = 0;            // 无左右移动
            Chassis_Cmd->w = 0;             // 无旋转
            break;
        case CHASSIS_DEAD:
            chassis_dead_control(Chassis_Cmd);
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
    float angle_hd = angle * PI / 180;              // 角度转弧度
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
    //麦轮运动学解算公式：
    // wheel0: 左前轮 = -vx - vy + (比例系数-1) * 轮子到中心距离 * wz
    // wheel1: 右前轮 =  vx - vy + (比例系数-1) * 轮子到中心距离 * wz
    // wheel2: 右后轮 =  vx + vy + (比例系数-1) * 轮子到中心距离 * wz
    // wheel3: 左后轮 = -vx + vy + (比例系数-1) * 轮子到中心距离 * wz
    Chassis_Cmd->wheel3508_rpm[0] = (Chassis_Cmd->vx - Chassis_Cmd->vy - MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w)*MPS_TO_MOTOR_RPM;
    Chassis_Cmd->wheel3508_rpm[1] = (-Chassis_Cmd->vx - Chassis_Cmd->vy - MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w)*MPS_TO_MOTOR_RPM;
    Chassis_Cmd->wheel3508_rpm[2] = (-Chassis_Cmd->vx + Chassis_Cmd->vy - MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w)*MPS_TO_MOTOR_RPM;
    Chassis_Cmd->wheel3508_rpm[3] = (Chassis_Cmd->vx + Chassis_Cmd->vy - MOTOR_DISTANCE_TO_CENTER * Chassis_Cmd->w)*MPS_TO_MOTOR_RPM;
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
    // 将计算出的轮子转速设置为各电机的目标速度
    for(int i = 0; i < 4; i++)
    {
        motors[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i];
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
    float current = motors[0].motor_data->feedback->pos / 22.75278; // 将编码器值转为角度值(0~360度)
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
    float SpinTop_Angle=0;
	SpinTop_Angle  = ((int32_t)motors[0].motor_data->feedback->pos - yaw_encoder_offset) % 8192 / 22.7555556f;//编码值转为角度值
	if(SpinTop_Angle > 360)
    {
		SpinTop_Angle = (SpinTop_Angle - 360);//2*pi/360
    }
    return SpinTop_Angle;
}



/**
 * @brief 检查底盘离线状态
 */
void Check_Chassis_Offline(void)
{
    if(Chassis_3508[0].motor_data->feedback->current == 0 || Chassis_3508[1].motor_data->feedback->current == 0 ||
       Chassis_3508[2].motor_data->feedback->current == 0 || Chassis_3508[3].motor_data->feedback->current == 0)
    {
        chassis_online = 0; // 如果有电机电流为0，认为底盘离线
    }
    else
    {
        chassis_online = 1; // 否则认为底盘在线
    }

}


/**
* @brief 死控运动阶段定义
*/
typedef struct
{
    int32_t duration_ticks;
    float vx;
    float vy;
    float w;
} Dead_Phase_s;

/**
 * @brief 底盘死控函数——按预定义阶段表依次执行运动，计数式非阻塞
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void chassis_dead_control(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    static uint8_t phase = 0;
    static int32_t tick = 0;
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;

    /* 检测模式从其他切换为CHASSIS_DEAD时复位 */
    if (last_mode != Chassis_Cmd->chassis_mode)
    {
        if (Chassis_Cmd->chassis_mode == CHASSIS_DEAD)
        {
            phase = 0;
            tick = 0;
        }
        last_mode = Chassis_Cmd->chassis_mode;
    }

    static const Dead_Phase_s dead_plan[] = {
        {1000,  0.0f,  0.0f,  0.0f},
        {1000,   200.0f,  0.0f,  0.0f},
        {1000,  0.0f,  100.0f,  0.0f},
        {500,   0.0f,  0.0f,  0.0f},
        {785,   0.0f,  0.0f,  0.0f},
        {500,   0.0f,  0.0f,  0.0f},
    };
    const uint8_t dead_phase_cnt = sizeof(dead_plan) / sizeof(dead_plan[0]);

    tick++;

    if (phase < dead_phase_cnt && tick >= dead_plan[phase].duration_ticks)
    {
        tick = 0;
        phase++;
    }

    if (phase < dead_phase_cnt)
    {
        Chassis_Cmd->vx = dead_plan[phase].vx * sensitivity;
        Chassis_Cmd->vy = dead_plan[phase].vy* sensitivity;
        Chassis_Cmd->w  = dead_plan[phase].w;
    }
    else
    {
        Chassis_Cmd->vx = 0.0f;
        Chassis_Cmd->vy = 0.0f;
        Chassis_Cmd->w  = 0.0f;
    }
}

#endif
