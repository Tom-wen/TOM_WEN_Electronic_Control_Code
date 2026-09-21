#include "Init.h"
#include "sentry_chassis.h"
#include "data_processing.h"

#ifdef AUTO_SENTRY

//底盘电机发送数据
Chassis_Ctrl_Cmd_s chassis_cmd_send;
Signal sig_rotate;

const float feed_forward_rpm_speed[4] = {0.0f, 0.00f, 0.00f, 0.0f};


/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 */
void chassis_task(void *argument)
{
    Chassis_Init(Chassis_3508, Chassis_6020 , &chassis_cmd_send);
    for(;;)
    {
        RemoteControlChassis(&chassis_cmd_send);
        Chassis_mode_update(&chassis_cmd_send);
        Chassis_control(Chassis_3508, Chassis_6020, &chassis_cmd_send);
        AGV_Speed_Feedback(Chassis_3508,Chassis_6020, &chassis_cmd_send);
        chassis_motor_updata(Chassis_3508, Chassis_6020);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 底盘初始化函数
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 */
void Chassis_Init(MotorInstance *motors1, MotorInstance *motors2 ,Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{   
    //底盘3508单环PID初始化
    PID_Init(&motors1[0].motor_data->pid[single_loop],  15.0f, 0.02f, 0.05f, 1000.0f, 20000.0f, 0.0f);
    PID_Init(&motors1[0].motor_data->pid[cascade_inner], 15.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors1[1].motor_data->pid[single_loop], 15.0f, 0.02f, 0.05f, 1000.0f, 10000.0f, 0.0f);
    PID_Init(&motors1[1].motor_data->pid[cascade_inner], 15.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[1].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors1[2].motor_data->pid[single_loop], 15.0f, 0.02f, 0.05f, 1000.0f, 10000.0f, 0.0f);
    PID_Init(&motors1[2].motor_data->pid[cascade_inner], 15.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[2].motor_data->pid[cascade_outer], 1.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors1[3].motor_data->pid[single_loop], 15.0f, 0.02f, 0.05f, 1000.0f, 10000.0f, 0.0f);
    PID_Init(&motors1[3].motor_data->pid[cascade_inner], 20.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors1[3].motor_data->pid[cascade_outer], 2.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    // 底盘6020PID初始化
    PID_Init(&motors2[0].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 50.0f, 0.0f, 1.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 5.0f, 0.0f, 0.0f, 20.0f, 1000.0f, 0.0f);

    PID_Init(&motors2[1].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[1].motor_data->pid[cascade_inner], 50.0f, 0.0f, 1.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors2[1].motor_data->pid[cascade_outer], 5.0f, 0.0f, 0.0f, 20.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors2[2].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[2].motor_data->pid[cascade_inner], 50.0f, 0.0f, 1.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors2[2].motor_data->pid[cascade_outer], 5.0f, 0.0f, 0.0f, 20.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors2[3].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors2[3].motor_data->pid[cascade_inner], 50.0f, 0.0f, 1.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors2[3].motor_data->pid[cascade_outer], 5.0f, 0.0f, 0.0f, 20.0f, 1000.0f, 0.0f);

    //底盘跟随云台旋转PID控制器初始化
    PID_Init(&Chassis_Cmd->wz_pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);//使用单级
    PID_Init(&Chassis_Cmd->wz_pid[cascade_inner], 0.5f, 0.0f, 0.0f, 20.0f, 100.0f, 0.0f);
    PID_Init(&Chassis_Cmd->wz_pid[cascade_outer], 0.5f, 0.0f, 0.0f, 10.0f, 50.0f, 0.0f);

    //底盘6020初始回零
    motors2[0].motor_data->target_position = Chassis_6020_offset_angle1;
    motors2[1].motor_data->target_position = Chassis_6020_offset_angle2;
    motors2[2].motor_data->target_position = Chassis_6020_offset_angle3;
    motors2[3].motor_data->target_position = Chassis_6020_offset_angle4;
    Chassis_Cmd->wheel3508_rpm[0] = 0.0f;
    Chassis_Cmd->wheel3508_rpm[1] = 0.0f;
    Chassis_Cmd->wheel3508_rpm[2] = 0.0f;
    Chassis_Cmd->wheel3508_rpm[3] = 0.0f;
    Chassis_Cmd->wheel6020_relative_angle[0] = 0.0f;
    Chassis_Cmd->wheel6020_relative_angle[1] = 0.0f;
    Chassis_Cmd->wheel6020_relative_angle[2] = 0.0f;
    Chassis_Cmd->wheel6020_relative_angle[3] = 0.0f;
    Chassis_Cmd->wheel6020_target_angle[0] = Chassis_6020_offset_angle1;
    Chassis_Cmd->wheel6020_target_angle[1] = Chassis_6020_offset_angle2;
    Chassis_Cmd->wheel6020_target_angle[2] = Chassis_6020_offset_angle3;
    Chassis_Cmd->wheel6020_target_angle[3] = Chassis_6020_offset_angle4;
    Chassis_Cmd->wheel6020_last_target_angle[0] = Chassis_6020_offset_angle1;
    Chassis_Cmd->wheel6020_last_target_angle[1] = Chassis_6020_offset_angle2;
    Chassis_Cmd->wheel6020_last_target_angle[2] = Chassis_6020_offset_angle3;
    Chassis_Cmd->wheel6020_last_target_angle[3] = Chassis_6020_offset_angle4;
    Chassis_Cmd->chassis_last_mode = CHASSIS_ZERO_FORCE;
    //初始化底盘速度
    chassis_motor_updata(motors1, motors2);

    //变速小陀螺参数赋值
    Signal_Init(&sig_rotate, 1.0f, 0.5f, 0.0f, 0.0f);
}

/**
 * @brief 设置底盘电机状态（启用/禁用）
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
    chassis_mode_e prev_mode = Chassis_Cmd->chassis_mode;

    #ifdef DJI_REMOTE
        switch (left_switch)
        {
        case switch_down:
            Chassis_Cmd->chassis_mode = CHASSIS_NO_FOLLOW;
            break; 
        case switch_mid:
            Chassis_Cmd->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case switch_up:
            Chassis_Cmd->chassis_mode = CHASSIS_AUTO;
            break;    
        default:
            Chassis_Cmd->chassis_mode = CHASSIS_ZERO_FORCE;
            break;
        }
    #endif

    if (prev_mode != Chassis_Cmd->chassis_mode)
    {
        Chassis_Cmd->chassis_last_mode = prev_mode;
    }
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
 * @brief 遥控器控制底盘速度
 * @param Chassis_Cmd 底盘控制命令结构体指针，包含遥控器输入数据和底盘控制模式
 */
void RemoteControlChassis(Chassis_Ctrl_Cmd_s *Chassis_Cmd) 
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }

    static float vx = 0.0f;
    static float vy = 0.0f;
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch(Chassis_Cmd->chassis_mode) 
    {
        case CHASSIS_FOLLOW_GIMBAL_YAW:                         //跟随云台
            Chassis_Cmd->vx =(float)rc_ctrl.rc.ch[1] * sensitivity ;
            Chassis_Cmd->vy =(float)rc_ctrl.rc.ch[0] * sensitivity;
            Chassis_Cmd->w = (Chassis_Cmd->wz_pid[single_loop].Out *0.05); // 使用PID输出作为角速度;
            break;
        case CHASSIS_NO_FOLLOW:                                 //不跟随云�?
            Chassis_Cmd->vx =(float)rc_ctrl.rc.ch[1]* sensitivity;
            Chassis_Cmd->vy =(float)rc_ctrl.rc.ch[0]* sensitivity;
            Chassis_Cmd->w = -(float)rc_ctrl.rc.ch[4]* sensitivity;
            break;
        case CHASSIS_ROTATE:		                            //小陀螺模�?
            Chassis_Cmd->vx = (float)rc_ctrl.rc.ch[1]* sensitivity;
            Chassis_Cmd->vy = (float)rc_ctrl.rc.ch[0]* sensitivity;
            Chassis_Cmd->w =-Signal_Sin_Gen(&sig_rotate,0.002f);
            break;
        case CHASSIS_AUTO:
            Chassis_Cmd->vx = cdc_proto_link.chassis_cmd.vx;
            Chassis_Cmd->vy = -cdc_proto_link.chassis_cmd.vy;//这里上位机
            Chassis_Cmd->w = cdc_proto_link.chassis_cmd.w; 
            break;
        case CHASSIS_DEAD:
            chassis_dead_control(Chassis_Cmd,Chassis_Cmd->chassis_last_mode);
            break;
	    case CHASSIS_ZERO_FORCE:		                        //零电流模�?
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
 * @param Chassis_6020 6020电机实例指针
 */
void AGV_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd , MotorInstance *Chassis_6020)
{
    if(Chassis_Cmd == NULL)
    {
        return;
    }
    static uint8_t target_inited = 0;
    static float last_target_angle[4];
    static uint8_t angle_inited[4] = {0};
    static float last_encoder_angle[4] = {0.0f};
    static float wheel6020_total_angle[4] = {0.0f};
    const float max_steer_step = 2.0f;
    const float offset_angle[4] =
    {
        Chassis_6020_offset_angle1,
        Chassis_6020_offset_angle2,
        Chassis_6020_offset_angle3,
        Chassis_6020_offset_angle4
    };
    float vw = Chassis_Cmd->w * Radius;

    if(target_inited == 0)
    {
        for(int i = 0; i < 4; i++)
        {
            last_target_angle[i] = offset_angle[i];
        }
        target_inited = 1;
    }

    for(int i = 0; i < 4; i++)
    {
        if(Chassis_6020[i].motor_data != NULL)
        {
            float current_encoder_angle = Chassis_6020[i].motor_data->feedback->pos / 22.75278f;

            if(angle_inited[i] == 0)
            {
                last_encoder_angle[i] = current_encoder_angle;
                wheel6020_total_angle[i] = current_encoder_angle;
                angle_inited[i] = 1;
            }
            else
            {
                float encoder_delta = current_encoder_angle - last_encoder_angle[i];

                if(encoder_delta > 180.0f)
                {
                    encoder_delta -= 360.0f;
                }
                else if(encoder_delta < -180.0f)
                {
                    encoder_delta += 360.0f;
                }

                wheel6020_total_angle[i] += encoder_delta;
                last_encoder_angle[i] = current_encoder_angle;
            }

            Chassis_Cmd->wheel6020_current_angle[i] = wheel6020_total_angle[i];
        }
    }

    //3508转速计算
    Chassis_Cmd->wheel3508_rpm[0] = -( sqrt(pow((Chassis_Cmd->vy - vw * cos45), 2) + pow((Chassis_Cmd->vx - vw * sin45), 2)) ) *GEAR_RATIO / WHEEL_RADIUS * 60.0f / (2.0f * PI);
    Chassis_Cmd->wheel3508_rpm[1] = ( sqrt(pow((Chassis_Cmd->vy - vw * cos45), 2) + pow((Chassis_Cmd->vx + vw * sin45), 2)) ) *GEAR_RATIO / WHEEL_RADIUS * 60.0f / (2.0f * PI);
    Chassis_Cmd->wheel3508_rpm[2] = ( sqrt(pow((Chassis_Cmd->vy + vw * cos45), 2) + pow((Chassis_Cmd->vx - vw * sin45), 2)) ) *GEAR_RATIO / WHEEL_RADIUS * 60.0f / (2.0f * PI);
    Chassis_Cmd->wheel3508_rpm[3] = -( sqrt(pow((Chassis_Cmd->vy + vw * cos45), 2) + pow((Chassis_Cmd->vx + vw * sin45), 2)) ) *GEAR_RATIO / WHEEL_RADIUS * 60.0f / (2.0f * PI);



    if(Chassis_Cmd->vx == 0 && Chassis_Cmd->vy == 0 && Chassis_Cmd->w == 0)
    {
        for(int i = 0; i < 4; i++)
        {
            Chassis_Cmd->wheel6020_target_angle[i] = last_target_angle[i];
            Chassis_Cmd->wheel6020_last_target_angle[i] = last_target_angle[i];
            Chassis_Cmd->wheel6020_relative_angle[i] = last_target_angle[i] - offset_angle[i];
        }
    }
    else
    {
        float raw_relative_angle[4];

        //6020要转角度
        raw_relative_angle[0] = atan2((Chassis_Cmd->vy - vw * cos45), (Chassis_Cmd->vx - vw * sin45)) * angle_change;
        raw_relative_angle[1] = atan2((Chassis_Cmd->vy - vw * cos45), (Chassis_Cmd->vx + vw * sin45)) * angle_change;
        raw_relative_angle[2] = atan2((Chassis_Cmd->vy + vw * cos45), (Chassis_Cmd->vx + vw * sin45)) * angle_change;
        raw_relative_angle[3] = atan2((Chassis_Cmd->vy + vw * cos45), (Chassis_Cmd->vx - vw * sin45)) * angle_change;

        for(int i = 0; i < 4; i++)
        {
            float raw_target_angle = offset_angle[i] + raw_relative_angle[i];
            float current_angle = last_target_angle[i];
            float desired_target_angle = 0.0f;
            float delta_from_last = 0.0f;
            float target_delta = 0.0f;
            float drive_scale = 1.0f;

            if(angle_inited[i] != 0)
            {
                current_angle = wheel6020_total_angle[i];
            }

            target_delta = raw_target_angle - current_angle;

            while(target_delta > 180.0f)
            {
                target_delta -= 360.0f;
            }
            while(target_delta < -180.0f)
            {
                target_delta += 360.0f;
            }

            // ROTATE 模式下跳过 90° 翻转：底盘自转时轮子方向连续扫过360°，
            // 翻转逻辑反复触发会导致底盘抽搐
            if(Chassis_Cmd->chassis_mode != CHASSIS_ROTATE)
            {
                if(target_delta > 90.0f)
                {
                    target_delta -= 180.0f;
                    Chassis_Cmd->wheel3508_rpm[i] = -Chassis_Cmd->wheel3508_rpm[i];
                }
                else if(target_delta < -90.0f)
                {
                    target_delta += 180.0f;
                    Chassis_Cmd->wheel3508_rpm[i] = -Chassis_Cmd->wheel3508_rpm[i];
                }
            }

            desired_target_angle = current_angle + target_delta;
            delta_from_last = desired_target_angle - last_target_angle[i];

            while(delta_from_last > 180.0f)
            {
                delta_from_last -= 360.0f;
            }
            while(delta_from_last < -180.0f)
            {
                delta_from_last += 360.0f;
            }

            if(delta_from_last > max_steer_step)
            {
                delta_from_last = max_steer_step;
            }
            else if(delta_from_last < -max_steer_step)
            {
                delta_from_last = -max_steer_step;
            }

            drive_scale = cosf(target_delta / angle_change);
            if(drive_scale < 0.0f)
            {
                drive_scale = 0.0f;
            }

            Chassis_Cmd->wheel3508_rpm[i] *= drive_scale;
            Chassis_Cmd->wheel6020_target_angle[i] = last_target_angle[i] + delta_from_last;
            Chassis_Cmd->wheel6020_relative_angle[i] = Chassis_Cmd->wheel6020_target_angle[i] - offset_angle[i];
            Chassis_Cmd->wheel6020_last_target_angle[i] = Chassis_Cmd->wheel6020_target_angle[i];
            last_target_angle[i] = Chassis_Cmd->wheel6020_target_angle[i];
        }
    }


}

/**
 * @brief 根据四舵轮电机反馈解算底盘实际速度
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 * @param Chassis_Cmd 底盘控制命令结构体指针，用于存储解算结果(vx, vy, w)
 */
void AGV_Speed_Feedback(MotorInstance *motors1, MotorInstance *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(motors1 == NULL || motors2 == NULL || Chassis_Cmd == NULL)
    {
        return;
    }

    const float rpm_to_ms = WHEEL_RADIUS / GEAR_RATIO * (2.0f * PI) / 60.0f;
    const float R_cos45 = (float)Radius * cos45;
    const float R_sin45 = (float)Radius * sin45;

    const float offset_angle[4] = 
    {
        Chassis_6020_offset_angle1,
        Chassis_6020_offset_angle2,
        Chassis_6020_offset_angle3,
        Chassis_6020_offset_angle4
    };

    float Vx[4], Vy[4];

    for(int i = 0; i < 4; i++)
    {
        float rpm = motors1[i].motor_data->feedback->vel;
        if(i == 0 || i == 3) rpm = -rpm;

        float speed = rpm * rpm_to_ms;

        float theta = (Chassis_Cmd->wheel6020_current_angle[i] - offset_angle[i]) / angle_change;

        Vx[i] = speed * cos(theta);
        Vy[i] = speed * sin(theta);
    }

    Chassis_Cmd->vx_fdb = (Vx[0] + Vx[1] + Vx[2] + Vx[3]) / 4.0f;
    Chassis_Cmd->vy_fdb = (Vy[0] + Vy[1] + Vy[2] + Vy[3]) / 4.0f;

    float w_L = (Vx[1] - Vx[0]) / (2.0f * R_sin45);
    float w_R = (Vx[3] - Vx[2]) / (2.0f * R_sin45);
    float w_F = (Vy[2] - Vy[0]) / (2.0f * R_cos45);
    float w_B = (Vy[3] - Vy[1]) / (2.0f * R_cos45);
    Chassis_Cmd->w_fdb = (w_L + w_R + w_F + w_B) / 4.0f;

    //将解析出来的数据传给上位机
    host_report.chassis_vx = Chassis_Cmd->vx_fdb;
    host_report.chassis_vy = Chassis_Cmd->vy_fdb;
    host_report.chassis_w = Chassis_Cmd->w_fdb;
}

/**
 * @brief 设置AGV电机目标速度和位置
 * @param motors1 3508电机实例指针
 * @param motors2 6020电机实例指针
 * @param Chassis_Cmd 底盘控制命令结构体指针，包含目标速度和位置数据
 */
void AGV_Motor_Set(MotorInstance  *motors1, MotorInstance  *motors2, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    if(motors1 == NULL || motors2 == NULL ||  Chassis_Cmd == NULL)
    {
        return;
    }
    for(int i = 0; i < 4; i++)
    {
        motors1[i].motor_data->target_velocity = Chassis_Cmd->wheel3508_rpm[i];
        motors2[i].motor_data->target_position = Chassis_Cmd->wheel6020_target_angle[i];
    }
}


/* 云台跟随 ---------------------------------------------------------------------- */

/**
 * @brief 计算底盘跟随云台旋转所需角速度
 * @param motors 云台电机实例数组指针
 * @param Chassis_Cmd 底盘控制命令结构体指针，包含底盘跟随云台旋转的PID控制器
 * @details 通过PID控制器使底盘跟随云台旋转，保持相对角度稳定在yaw_offset设定值。当前角度通过云台电机的编码器反馈计算得到，目标角度为yaw_offset。PID控制器输出底盘需要的角速度来调整底盘姿态，使其跟随云台旋转。
 */
void Set_follow_wz(MotorInstance  *motors, Chassis_Ctrl_Cmd_s *Chassis_Cmd)
{
    const float max_angle = 360;                            // 最大角度值
    float current = motors[0].motor_data->feedback->pos *hudu +180.0f ; // 将编码器值转为角度
    float target = yaw_offset;                              // 目标角度为偏航角偏移
    // 处理角度跨越问题，确保目标角度与当前角度在同一连续区间
    while (target - current > max_angle / 2) 
    {
        target -= max_angle;
    } 
    while (target - current < -max_angle / 2) 
    {
        target += max_angle;
    }
    // 角度死区判断：误差小于阈值时直接清零输出
    if(fabsf(target - current) < WZ_ANGLE_DEADZONE)
    {
        Chassis_Cmd->wz_pid[single_loop].Out = 0.0f;
        return;
    }
    // 使用单级PID控制器计算跟随所需的角速度
    PID_Calc(&Chassis_Cmd->wz_pid[single_loop], target, current);
    // PID输出死区处理
    if(fabsf(Chassis_Cmd->wz_pid[single_loop].Out) < WZ_PID_DEADZONE)
    {
        Chassis_Cmd->wz_pid[single_loop].Out = 0.0f;
    }
}


/* 坐标变换 ---------------------------------------------------------------------- */

/**
  * @brief  将云台坐标系的速度转换为底盘坐标系的速度
  * @param  Chassis_Cmd 底盘控制命令结构体指针，包含云台坐标系下的速度(vx, vy)和角速度(w)
  * @param  angle 云台相对于底盘的角度（单位：度）
  * @retval 无
  * @attention 输入的速度应在云台坐标系下
  */
void Absolute_Cal(Chassis_Ctrl_Cmd_s *Chassis_Cmd, float angle)
{
    float angle_hd = angle * PI / 180;              // 角度转弧度
    Chassis_Ctrl_Cmd_s temp_speed;                  // 临时速度变量
    temp_speed.w = Chassis_Cmd->w;              // 放大角速度
    temp_speed.chassis_mode = Chassis_Cmd->chassis_mode; // 传递模式供AGV_Cal使用
    // 坐标变换：将云台坐标系的速度分量转换到底盘坐标系
    temp_speed.vx = Chassis_Cmd->vx * cos(angle_hd) - Chassis_Cmd->vy * sin(angle_hd);
    temp_speed.vy = Chassis_Cmd->vx * sin(angle_hd) + Chassis_Cmd->vy * cos(angle_hd);                      // 计算转换后的底盘电机速度
    AGV_Cal(&temp_speed,Chassis_6020);
    // 将计算结果复制回原结构体
    for(int i = 0; i < 4; i++)
    {
        Chassis_Cmd->wheel3508_rpm[i] = temp_speed.wheel3508_rpm[i] + feed_forward_rpm_speed[i];
        Chassis_Cmd->wheel6020_target_angle[i] = temp_speed.wheel6020_target_angle[i];
    }
}


/**
 * @brief 获取底盘相对云台偏航角的旋转角度
 * @param motors 云台电机实例指针
 * @return 底盘相对云台偏航角的旋转角度（度）(这里是4310也就是大yaw轴编码器的角度，对于4310反馈是-pi-pi之间，对应角度是-180-180之间，因此换算后的角度应该加上180)
 */
float Chassis_relative_angle(MotorInstance  *motors)
{
    float SpinTop_Angle=0;
	SpinTop_Angle  = (motors[0].motor_data->feedback->pos * hudu +180.0f - yaw_offset);//编码值转为角度
	if(SpinTop_Angle > 360)
    {
		SpinTop_Angle = (SpinTop_Angle - 360);
    }
    return SpinTop_Angle;
}


/**
 * @brief 底盘死控函数——按预定义阶段表依次执行运动，计数式非阻塞
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void chassis_dead_control(Chassis_Ctrl_Cmd_s *Chassis_Cmd , chassis_mode_e last_mode)
{
    static uint8_t phase = 0;
    static int32_t tick = 0;//这里经过测试发现一个tick为4ms
    /* 检测模式从其他切换为CHASSIS_DEAD时复位 */
    if (last_mode != Chassis_Cmd->chassis_mode)
    {
        if (Chassis_Cmd->chassis_mode == CHASSIS_DEAD)
        {
            phase = 0;
            tick = 0;
        }
    }
    tick++;

    if (phase < DEAD_PLAN_CNT && tick >= dead_plan[phase].duration_ticks)
    {
        tick = 0;
        phase++;
    }

    if (phase < DEAD_PLAN_CNT)
    {
        Chassis_Cmd->vx = dead_plan[phase].vx;
        Chassis_Cmd->vy = dead_plan[phase].vy;
        Chassis_Cmd->w  = dead_plan[phase].w;
    }
    else
    {
        Chassis_Cmd->vx = 0.0f;
        Chassis_Cmd->vy = 0.0f;
        Chassis_Cmd->w  = 0.0f;
    }
}

/**
 * @brief 底盘控制主函数——根据当前模式选择对应的控制策略
 * @param Motor1 3508电机实例指针
 * @param Motor2 6020电机实例指针
 * @param Chassis_Cmd 底盘控制命令结构体指针
 */
void Chassis_control(MotorInstance*Motor1, MotorInstance*Motor2, Chassis_Ctrl_Cmd_s*Chassis_Cmd)
{
        switch (Chassis_Cmd->chassis_mode)
        {
            case CHASSIS_ZERO_FORCE:
                Chassis_Motor_Status(Motor_Disable, Motor1, Motor2, 4);
                break;
            case CHASSIS_NO_FOLLOW:
                Chassis_Motor_Status(Motor_Enable, Motor1, Motor2, 4);
                AGV_Cal(Chassis_Cmd ,Motor2 );
                AGV_Motor_Set(Motor1, Motor2, Chassis_Cmd);
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                Chassis_Motor_Status(Motor_Enable, Motor1, Motor2, 4);
                Set_follow_wz(Gimbal_Big_Yaw_4310, Chassis_Cmd);
                AGV_Cal(Chassis_Cmd ,Motor2 );
                AGV_Motor_Set(Motor1, Motor2, Chassis_Cmd);
                break;
            case CHASSIS_ROTATE:
                Chassis_Motor_Status(Motor_Enable, Motor1, Motor2, 4);
                Absolute_Cal(Chassis_Cmd, Chassis_relative_angle(Gimbal_Big_Yaw_4310));
                AGV_Motor_Set(Motor1, Motor2, Chassis_Cmd);
                break;
            case  CHASSIS_AUTO:
                Chassis_Motor_Status(Motor_Enable, Motor1, Motor2, 4);
                Absolute_Cal(Chassis_Cmd, Chassis_relative_angle(Gimbal_Big_Yaw_4310));
                AGV_Motor_Set(Motor1, Motor2, Chassis_Cmd);
                break;
            case CHASSIS_DEAD:
                Chassis_Motor_Status(Motor_Enable, Motor1, Motor2, 4);
                AGV_Cal(Chassis_Cmd ,Motor2 );
                AGV_Motor_Set(Motor1, Motor2, Chassis_Cmd);                         
        }
}


#endif

