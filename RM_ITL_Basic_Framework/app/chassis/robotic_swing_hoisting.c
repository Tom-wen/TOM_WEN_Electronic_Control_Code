#include "robotic_swing_hoisting.h"
#include "dm_imu.h"

#ifdef ROBOTIC_SWING_HOISTING
Hoisting_Ctrl_Cmd_s hoisting_cmd_send;
uint16_t loop_count = 0;

void hoisting_task(void *argument)
{
    Hoisting_Init(Hoisting_5047, &hoisting_cmd_send);
    for(;;)
    {
        Hoisting_mode_update(&hoisting_cmd_send); //抬升数据的相关更新
        switch (hoisting_cmd_send.hoisting_mode)
        {
            case HOISTING_MOTOR_OFF:
                Hoisting_Motor_Status(Motor_Disable, Hoisting_5047, 4);
                Hoisting_control(Hoisting_5047, &hoisting_cmd_send);
            break;

            case SET_OFF_HOISTING:
                Hoisting_Motor_Status(Motor_Enable, Hoisting_5047, 4);
                Hoisting_control(Hoisting_5047, &hoisting_cmd_send);
            break;

            case SET_UP_HOISTING:
                Hoisting_Motor_Status(Motor_Enable, Hoisting_5047, 4);
                Hoisting_control(Hoisting_5047, &hoisting_cmd_send);
            break;

            case SET_UP_SWING_HOISTING:
                Hoisting_Motor_Status(Motor_Enable, Hoisting_5047, 4);
                Hoisting_control(Hoisting_5047, &hoisting_cmd_send);
            break;

            case AUTO_LEVEL_ADJUST:
                Hoisting_Motor_Status(Motor_Enable, Hoisting_5047, 4);
                Hoisting_control(Hoisting_5047, &hoisting_cmd_send);
            break;
            
            default:
            break;
        }
        Hoisting_motor_updata(Hoisting_5047);
        // 控制任务执行频率（2ms周期）
        vTaskDelay(pdMS_TO_TICKS(2));
    }

}

/**
 * @brief 底盘抬升初始化函数
 * @param motors 底盘电机实例数组指针
 * @param Hoisting_Cmd 底盘抬升控制命令结构体指针
 * @details 初始化底盘各电机的PID控制器参数
 */
void Hoisting_Init(MotorInstance *motors, Hoisting_Ctrl_Cmd_s *Hoisting_Cmd)
{   
    //PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 30.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 50.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], 30.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 50.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[2].motor_data->pid[single_loop], 30.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_inner], 50.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[2].motor_data->pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);
    
    PID_Init(&motors[3].motor_data->pid[single_loop], 30.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_inner], 50.0f, 0.0f, 0.0f, 10.0f, 15000.0f, 0.0f);
    PID_Init(&motors[3].motor_data->pid[cascade_outer], 10.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.0f);

    Hoisting_Cmd->backward_hoisting = 0.0f;
    Hoisting_Cmd->forward_hoisting = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        Hoisting_Cmd->motor_normalized_pos[i] = 0.0f;
        Hoisting_Cmd->motor_revolution_count[i] = 0;
    }

    GQ_Motor_Init(motors);//高擎电机初始化使能

    PID_Init(&Hoisting_Cmd->leveling_pitch_pid, LEVELING_PID_KP, LEVELING_PID_KI, LEVELING_PID_KD, LEVELING_PID_MAX_I, LEVELING_PID_MAX_OUT, 0.0f);
    PID_Init(&Hoisting_Cmd->leveling_roll_pid,  LEVELING_PID_KP, LEVELING_PID_KI, LEVELING_PID_KD, LEVELING_PID_MAX_I, LEVELING_PID_MAX_OUT, 0.0f);
    Hoisting_Cmd->pitch_adjustment = 0.0f;
    Hoisting_Cmd->roll_adjustment = 0.0f;
}

/**
 * @brief 更新底盘工作模式
 * @param Chassis_Cmd 底盘控制命令结构体指针
 * @details 根据遥控器开关状态更新底盘工作模式
 */
void Hoisting_mode_update(Hoisting_Ctrl_Cmd_s *Hoisting_Cmd)
{
    if(Hoisting_Cmd == NULL)
    {
        return;
    }
    // 如果SBUS信号断开，则强制进入零力模式
    if(sbus_online == 0)
    {
        Hoisting_Cmd->hoisting_mode = HOISTING_MOTOR_OFF;
        for (int i = 0; i < 4; i++)
        {
            Hoisting_Cmd->motor_init_angle[i] = Hoisting_5047[i].motor_data->feedback->pos*2*PI;//弧度
        }
        return;
    }
    // if(chassis_online == 0)
    // {
    //     GQ_Motor_Init(Hoisting_5047);
    //     for (int i = 0; i < 4; i++)
    //     {
    //         Hoisting_Cmd->motor_init_angle[i] = Hoisting_5047[i].motor_data->feedback->pos*2*PI;//弧度
    //     }
    //     return;
    // }
    #ifdef DJI_REMOTE
    hoisting_mode_e prev_mode = Hoisting_Cmd->hoisting_mode;
    // if (side_switch_on)
    // {
    //     Hoisting_Cmd->hoisting_mode = SET_UP_SWING_HOISTING;
    // }
    // else
    // {
    //     Hoisting_Cmd->hoisting_mode = SET_OFF_HOISTING;
    // }
    // if (prev_mode != Hoisting_Cmd->hoisting_mode && Hoisting_Cmd->hoisting_mode == AUTO_LEVEL_ADJUST)
    // {
    //     PID_Clear(&Hoisting_Cmd->leveling_pitch_pid);
    //     PID_Clear(&Hoisting_Cmd->leveling_roll_pid);
    // }
    #endif


}

/**
 * @brief 抬升电机状态设置函数
 * @param[in] status 电机状态（使能或失能）
 * @param[in] motors 电机实例数组指针
 * @param[in] motor_count 电机数量
 * @details 根据指定状态批量设置抬升电机的使能状态
 */
void Hoisting_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
{
    switch (status)
    {
        case Motor_Enable:
            // 使能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                GQ_Motor_enable(motors[i].motor_data);
            }
            break;
            
        case Motor_Disable:
            // 失能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                GQ_Motor_disable(motors[i].motor_data);
            }
            break;
            
        default:
            // 默认情况下失能所有电机
            for(int i = 0; i < motor_count; i++)
            {
                GQ_Motor_disable(motors[i].motor_data);
            }
            break;
    }
}


void Hoisting_control(MotorInstance *Hoisting_5047, Hoisting_Ctrl_Cmd_s *Hoisting_Cmd)
{
    float raw_pos_rev, norm_pos_rev;
    float abs_target_rev, norm_target_rev;
    int32_t rev_count;

    for (int i = 0; i < 4; i++)
    {
        raw_pos_rev = Hoisting_5047[i].motor_data->feedback->pos;
        rev_count = (int32_t)roundf(raw_pos_rev);
        norm_pos_rev = raw_pos_rev - rev_count;
        Hoisting_Cmd->motor_normalized_pos[i] = norm_pos_rev;
        Hoisting_Cmd->motor_revolution_count[i] = rev_count;
    }

    switch (hoisting_cmd_send.hoisting_mode)
    {
        case HOISTING_MOTOR_OFF:
            // 归一化反馈已在 switch 之前完成，此处仅保持圈数和位置持续跟踪，不输出角度控制
        break;

        case SET_OFF_HOISTING:
            Hoisting_Cmd->motor_target_angle[0] = Hoisting_Cmd->motor_init_angle[0];
            Hoisting_Cmd->motor_target_angle[1] = Hoisting_Cmd->motor_init_angle[1];
            Hoisting_Cmd->motor_target_angle[2] = Hoisting_Cmd->motor_init_angle[2];
            Hoisting_Cmd->motor_target_angle[3] = Hoisting_Cmd->motor_init_angle[3];
            for (int i = 0; i < 4; i++)
            {
                abs_target_rev = Hoisting_Cmd->motor_target_angle[i] / (2.0f * PI);
                norm_target_rev = abs_target_rev - Hoisting_Cmd->motor_revolution_count[i];

                PID_CascadeCalc(Hoisting_5047[i].motor_data->pid, norm_target_rev * 2.0f * PI, Hoisting_Cmd->motor_normalized_pos[i] * 2.0f * PI, Hoisting_5047[i].motor_data->feedback->vel);
                Hoisting_5047[i].motor_data->target_current = Hoisting_5047[i].motor_data->pid[cascade_inner].Out;
            }
        break;
            case SET_UP_SWING_HOISTING:
            {
                
                static hoisting_mode_e prev_mode = HOISTING_MOTOR_OFF;
                if (prev_mode != SET_UP_SWING_HOISTING)
                {
                    loop_count = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        Hoisting_Cmd->motor_init_angle[i] = Hoisting_5047[i].motor_data->feedback->pos * 2.0f * PI;
                    }
                }
                prev_mode = SET_UP_SWING_HOISTING;

                const uint16_t total_cycle = 2 * switchtime;
                loop_count++;
                if (loop_count >= total_cycle) loop_count = 0;

                if (loop_count < switchtime)
                {
                    Hoisting_Cmd->motor_target_angle[0] = Hoisting_Cmd->motor_init_angle[0]-HOISTING_AUTO_RIGHT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[1] = Hoisting_Cmd->motor_init_angle[1]+HOISTING_AUTO_LEFT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[2] = Hoisting_Cmd->motor_init_angle[2]-HOISTING_AUTO_LEFT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[3] = Hoisting_Cmd->motor_init_angle[3]+HOISTING_AUTO_RIGHT_ANGLE_OFFSET;
                }
                else
                {
                    Hoisting_Cmd->motor_target_angle[0] = Hoisting_Cmd->motor_init_angle[0]-HOISTING_AUTO_LEFT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[1] = Hoisting_Cmd->motor_init_angle[1]+HOISTING_AUTO_RIGHT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[2] = Hoisting_Cmd->motor_init_angle[2]-HOISTING_AUTO_RIGHT_ANGLE_OFFSET;
                    Hoisting_Cmd->motor_target_angle[3] = Hoisting_Cmd->motor_init_angle[3]+HOISTING_AUTO_LEFT_ANGLE_OFFSET;
                }
                for (int i = 0; i < 4; i++)
                {
                    abs_target_rev = Hoisting_Cmd->motor_target_angle[i] / (2.0f * PI);
                    norm_target_rev = abs_target_rev - Hoisting_Cmd->motor_revolution_count[i];

                    PID_CascadeCalc(Hoisting_5047[i].motor_data->pid, norm_target_rev * 2.0f * PI,
                    Hoisting_Cmd->motor_normalized_pos[i] * 2.0f * PI, Hoisting_5047[i].motor_data->feedback->vel);
                    Hoisting_5047[i].motor_data->target_current = Hoisting_5047[i].motor_data->pid[cascade_inner].Out;
                }
            break;
        }

        case SET_UP_HOISTING:
            Hoisting_Cmd->motor_target_angle[0] = Hoisting_Cmd->motor_init_angle[0]-HOISTING_AUTO_FORWARD_ANGLE_OFFSET;
            Hoisting_Cmd->motor_target_angle[1] = Hoisting_Cmd->motor_init_angle[1]+HOISTING_AUTO_FORWARD_ANGLE_OFFSET;
            Hoisting_Cmd->motor_target_angle[2] = Hoisting_Cmd->motor_init_angle[2]-HOISTING_AUTO_BACKWARD_ANGLE_OFFSET;
            Hoisting_Cmd->motor_target_angle[3] = Hoisting_Cmd->motor_init_angle[3]+HOISTING_AUTO_BACKWARD_ANGLE_OFFSET;
            for (int i = 0; i < 4; i++)
            {
                abs_target_rev = Hoisting_Cmd->motor_target_angle[i] / (2.0f * PI);
                norm_target_rev = abs_target_rev - Hoisting_Cmd->motor_revolution_count[i];

                PID_CascadeCalc(Hoisting_5047[i].motor_data->pid, norm_target_rev * 2.0f * PI, Hoisting_Cmd->motor_normalized_pos[i] * 2.0f * PI, Hoisting_5047[i].motor_data->feedback->vel);
                Hoisting_5047[i].motor_data->target_current = Hoisting_5047[i].motor_data->pid[cascade_inner].Out;
            }

        break;

        case AUTO_LEVEL_ADJUST:
        {
            if (fabsf(imu.pitch) < LEVELING_DEAD_ZONE_DEG && fabsf(imu.roll) < LEVELING_DEAD_ZONE_DEG)
            {
                Hoisting_Cmd->pitch_adjustment = 0.0f;
                Hoisting_Cmd->roll_adjustment  = 0.0f;
                PID_Clear(&Hoisting_Cmd->leveling_pitch_pid);
                PID_Clear(&Hoisting_Cmd->leveling_roll_pid);
            }
            else
            {
                PID_Calc(&Hoisting_Cmd->leveling_pitch_pid, 0.0f, imu.pitch);
                PID_Calc(&Hoisting_Cmd->leveling_roll_pid,  0.0f, imu.roll);
                Hoisting_Cmd->pitch_adjustment = Hoisting_Cmd->leveling_pitch_pid.Out;
                Hoisting_Cmd->roll_adjustment  = Hoisting_Cmd->leveling_roll_pid.Out;
            }

            float pitch_sign[4] = {+1.0f, +1.0f, -1.0f, -1.0f};
            float roll_sign[4]  = {+1.0f, -1.0f, +1.0f, -1.0f};

            for (int i = 0; i < 4; i++)
            {
                float pitch_correction = pitch_sign[i] * Hoisting_Cmd->pitch_adjustment;
                float roll_correction  = roll_sign[i]  * Hoisting_Cmd->roll_adjustment;
                Hoisting_Cmd->motor_target_current[i] = pitch_correction + roll_correction;
            }

            for (int i = 0; i < 4; i++)
            {
                if(fabs(Hoisting_5047[i].motor_data->feedback->pos) > 2.0f)
                {
                    Hoisting_5047[i].motor_data->target_current = 0.0f;
                }
                else
                {
                    Hoisting_5047[i].motor_data->target_current = Hoisting_Cmd->motor_target_current[i];
                }
            }
        }
        break;
      


        default:
        break;

    }

}

/**
 * @brief 更新底盘抬升电机控制
 * @param motors 底盘抬升电机实例数组指针
 * @details 执行电机控制算法并更新电机输出
 */
void Hoisting_motor_updata(MotorInstance *motors)
{
    if(motors == NULL)
    {
        return;
    }
    motors->motor_control(motors);
}

#endif