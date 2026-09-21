#include "Init.h"
#include "target_chassis.h"

#ifdef TARGET_CHASSIS

Chassis_Ctrl_Cmd_s chassis_cmd_send;
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
/**
 * @brief 靶车主任务函数
 * @param argument 任务参数
 * @details 这是底盘控制的主任务函数，负责根据不同的模式控制底盘运动
 */
void chassis_task(void *argument)
{
    Target_Chassis_PID_Init(Chassis_3508, Gimbal_5047);
    for(;;)
    {
        RemoteControlChassis(&chassis_cmd_send, &gimbal_cmd_send , Chassis_3508, Gimbal_5047);
        Chassis_mode_update(&chassis_cmd_send);
        Target_control(Chassis_3508, &chassis_cmd_send, Gimbal_5047, &gimbal_cmd_send);
        Motor_Set(Chassis_3508, &chassis_cmd_send, Gimbal_5047, &gimbal_cmd_send);
        chassis_motor_update(Chassis_3508, Gimbal_5047);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 靶车初始化函数
 * @param motors1 3508电机实例指针
 * @param motors2 5047电机实例指针
 */
void Target_Chassis_PID_Init(MotorInstance *motors1, MotorInstance *motors2)
{
    if(motors1 == NULL || motors2 == NULL)
    {
        return;
    }
    //底盘3508单环PID初始化
    PID_Init(&motors1[0].motor_data->pid[single_loop],  15.0f, 0.02f, 0.05f, 1000.0f, 20000.0f, 0.0f);
    PID_Init(&motors1[0].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors1[1].motor_data->pid[single_loop], 15.0f, 0.02f, 0.05f, 1000.0f, 10000.0f, 0.0f);
    PID_Init(&motors1[1].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[1].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    //云台5047单环PID初始化
    PID_Init(&motors2[0].motor_data->pid[single_loop], 15.0f, 0.02f, 0.05f, 1000.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 15.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    GQ_Motor_Init(motors2);//高擎电机初始化使能
    //初始化底盘速度
    chassis_motor_update(motors1, motors2);
}


/**
 * @brief 遥控器控制靶车
 * @param Chassis_Cmd 底盘控制命令结构体指针，包含遥控器输入数据和底盘控制模式
 * @param Gimbal_Cmd 云台控制命令结构体指针，包含云台控制数据
 */
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd , Gimbal_Ctrl_Cmd_s *Gimbal_Cmd,MotorInstance *Chassis_3508,MotorInstance *Gimbal_5047)
{
    if(Chassis_Cmd == NULL || Gimbal_Cmd == NULL)
    {
        return;
    }
    switch(Chassis_Cmd->chassis_mode) 
    {
        case CHASSIS_ROTATE:		                            //小陀螺模�?
            Chassis_Cmd->v = (float)rc_ctrl.rc.ch[1]* sensitivity;
            Gimbal_Cmd->w = -w_z;
            Motor_Status(Motor_Enable,Chassis_3508, Gimbal_5047, 2,1);
            break;
        case CHASSIS_FOLLOW_GIMBAL:
            Chassis_Cmd->v = 0.0f;
            Gimbal_Cmd->w = -w_z;
            Motor_Status(Motor_Enable,Chassis_3508, Gimbal_5047, 2,1);
            break;
	    case CHASSIS_ZERO_FORCE:		                        //零电流模�?
            Chassis_Cmd->v = 0;
            Gimbal_Cmd->w = 0;
            Motor_Status(Motor_Disable,Chassis_3508, Gimbal_5047, 2,1);
            break;
        default:
            break;
    }
}

/**
 * @brief 更新底盘模式
 * @param Chassis_Cmd 底盘控制命令结构体指�?
 */
void Chassis_mode_update(Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    if(sbus_online == 0)
    {
        Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }
    /* 检测模式 */
    if (Chassis_Cmd->chassis_last_mode != Chassis_Cmd->chassis_mode)
    {
        Chassis_Cmd->chassis_last_mode = Chassis_Cmd->chassis_mode;
    }
 
    #ifdef DJI_REMOTE
        switch (left_switch)
        {
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break; 
        case switch_mid:
            Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
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
 * @brief 控制主函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Chassis_Cmd 控制命令结构体指针
 */
void Target_control(MotorInstance *motor_chassis,Chassis_Ctrl_Cmd_s *Chassis_Cmd,MotorInstance *motors_gimbal,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd)
{
    if(Chassis_Cmd == NULL || Gimbal_Cmd == NULL)
    {
        return;
    }
    // 根据标志位选择控制模式
    if(Chassis_Cmd->chassis_mode == CHASSIS_FOLLOW_GIMBAL )
    {
    }
    else if(Chassis_Cmd->chassis_mode == CHASSIS_ROTATE)
    {
        Chassis_Cmd->wheel3508_rpm[0] = Chassis_Cmd->v *MPS_TO_MOTOR_RPM;
        Chassis_Cmd->wheel3508_rpm[1] = Chassis_Cmd->v *MPS_TO_MOTOR_RPM;

        if(motors_gimbal == NULL || motors_gimbal[0].motor_data == NULL || motors_gimbal[0].motor_data->feedback == NULL)
        {
            return;
        }
        PID_Calc(&motors_gimbal[0].motor_data->pid[single_loop], Gimbal_Cmd->w *(GIMBAL_GEAR_RATIO), motors_gimbal[0].motor_data->feedback->vel*REV_TO_RADS);
        Gimbal_Cmd->target_current = motors_gimbal[0].motor_data->pid[single_loop].Out;
    }
    else
    {         
    }
}

/**
 * @brief 设置电机状态（启用/禁用）
 * @param status 电机状态（启用/禁用）
 * @param motors 3508电机实例指针
 * @param motors1 5047电机实例指针
 * @param motor_count 电机数量
 */
void Motor_Status(Motor_status status, MotorInstance *motors,  MotorInstance *motors1, uint8_t motor1_count ,uint8_t motor2_count)
{
    if(motors == NULL || motors1 == NULL)
    {
        return;
    }
    switch (status)
    {
    case Motor_Enable:
        for(int i = 0; i < motor1_count; i++)
        {
            DJI_Motor_enable(motors[i].motor_data);
        }
        for(int i = 0; i < motor2_count; i++)
        {
            GQ_Motor_enable(motors1[i].motor_data);
        }
        break;
    case Motor_Disable:
        for(int i = 0; i < motor1_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
        }
        for(int i = 0; i < motor2_count; i++)
        {
            GQ_Motor_disable(motors1[i].motor_data);
        }
        break;
    default:
        for(int i = 0; i < motor1_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
        }
        for(int i = 0; i < motor2_count; i++)
        {
            GQ_Motor_disable(motors1[i].motor_data);
        }
        break;
    }
}

void Motor_Set(MotorInstance *motor_chassis,Chassis_Ctrl_Cmd_s *Chassis_Cmd,MotorInstance *motors_gimbal,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd)
{
    if(motor_chassis == NULL || motors_gimbal == NULL || Chassis_Cmd == NULL || Gimbal_Cmd == NULL)
    {
        return;
    }
    if(motor_chassis[0].motor_data == NULL || motor_chassis[1].motor_data == NULL || motors_gimbal[0].motor_data == NULL)
    {
        return;
    }
    motor_chassis[0].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[0];
    motor_chassis[1].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[1];
    motors_gimbal[0].motor_data->target_current = Gimbal_Cmd->target_current;
    Gimbal_Cmd->fdb_w = motors_gimbal[0].motor_data->feedback->vel*REV_TO_RADS/(GIMBAL_GEAR_RATIO);
}

/**
 * @brief 更新电机控制数据
 * @param motors1 3508电机实例指针
 * @param motors2 5047电机实例指针
 */
void chassis_motor_update(MotorInstance *motors1, MotorInstance *motors2)
{
    if(motors1 == NULL || motors2 == NULL)
    {
        return;
    }
    //底盘3508电机给电流
    if(motors1->motor_control != NULL)
    {
        motors1->motor_control(motors1);
    }
    //底盘5047电机给电流
    if(motors2->motor_control != NULL)
    {
        motors2->motor_control(motors2);
    }
}
#endif