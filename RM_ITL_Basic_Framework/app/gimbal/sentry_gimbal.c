#include "Init.h"
#include "sentry_gimbal.h"
#include "sentry_chassis.h"
#include "data_processing.h"

#ifdef AUTO_SENTRY
/**
 * @brief 云台控制命令结构体
 * @details 用于存储云台控制的各种参数和命令
 */
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;

/* 任务与初始化 ------------------------------------------------------------------ */

/**
 * @brief 云台主任务函数
 * @param argument 任务参数
 */
void gimbal_task(void *argument)
{
    Gimbal_Init(Gimbal_Small_Yaw_6020, Gimbal_Big_Yaw_4310 ,Gimbal_Pitch_4310);  
    for(;;)
    {
        // 处理遥控器输入数据，映射到云台控制命令
        RemoteControlGimbal(&gimbal_cmd_send);
        Gimbal_mode_update(&gimbal_cmd_send);
        Gimbal_control(Gimbal_Small_Yaw_6020, Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, &gimbal_cmd_send, &cdc_proto_link, &INS);
        gimbal_motor_updata(Gimbal_Small_Yaw_6020, Gimbal_Big_Yaw_4310, Gimbal_Pitch_4310);
        Big_Yaw_Chassis_Relative_Angle_Update(Gimbal_Big_Yaw_4310);
        Motor4310_detect_enable();//这个函数用于达妙电机离线时的使能
        vTaskDelay(pdMS_TO_TICKS(2));

    }

}


/* 电机控制 ---------------------------------------------------------------------- */

/**
 * @brief 云台电机状态更新函数
 * @param[in] motors 电机实例数组指针
 * @details 调用电机控制函数，将控制指令发送给电机
 */
void gimbal_motor_updata(MotorInstance *motors1, MotorInstance *motors2, MotorInstance *motors3)
{
    // 参数有效性检查
    if(motors1 == NULL || motors2 == NULL || motors3 == NULL)
    {
        return;
    }
    
    // 调用电机控制函数，更新电机状态
    motors1[0].motor_control(&motors1[0]);
    motors2[0].motor_control(&motors2[0]);
    motors3[0].motor_control(&motors3[0]);
}

/**
 * @brief 达妙电机离线检测使能
 * @details 检测大YAW轴4310电机是否在线，若在线则使能PITCH轴4310电机
 */
void Motor4310_detect_enable(void)
{
    Motor4310_enable_judge(Gimbal_Big_Yaw_4310);//4310电机使能
    if(Gimbal_Big_Yaw_4310[0].motor_data->feedback->tor!=0.0f)
    {
        Motor4310_enable_judge(Gimbal_Pitch_4310);
    }
}


/* 初始化 ------------------------------------------------------------------------ */

/**
 * @brief 云台初始化函数
 * @param[in] motors 电机实例数组指针
 * @details 初始化云台电机的PID控制器参数
 *          包括单环PID、串级PID的内环和外环
 */
void Gimbal_Init(MotorInstance *motors_1, MotorInstance *motors_2, MotorInstance *motors_3)
{
    // 小YAW轴6020电机PID初始化
    PID_Init(&motors_1[0].motor_data->pid[single_loop], 0.07f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors_1[0].motor_data->pid[cascade_inner], 12.0f, 0.0f, 0.0f, 1.0f, 15000.0f, 0.0f);
    PID_Init(&motors_1[0].motor_data->pid[cascade_outer], 12.0f, 0.0f, 0.0f, 1.0f, 15000.0f, 0.0f);
             
    // 大yaw轴4310电机PID初始化
    PID_Init(&motors_2[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors_2[0].motor_data->pid[cascade_inner], 0.08f, 0.0f, 0.05f, 10.0f, 200.0f, 0.0f);
    PID_Init(&motors_2[0].motor_data->pid[cascade_outer], 0.9f, 0.0f, 0.01f, 5.0f, 200.0f, 0.0f);

    MPC_Init(&motors_2[0].motor_data->mpc[mpc_single_loop],
    10, 10,     //预测/控制时域
    1.0f, 0.002f,
    0.0f, 0.9837656f,  // 2×2 状态矩阵
    0.0f, -0.0010383f,   // 2×1 输入矩阵
    30000.0f, 80.0f , 0.0010f, 0.0f, // 权重 + 前馈
    3500.0f, 25000.0f, 0.002f,  // 积分/输出限幅/采样时间(与 gimbal_task 2ms 一致)
    0.08f); //死区

    //pitch轴4310电机PID初始化
    PID_Init(&motors_3[0].motor_data->pid[single_loop], 0.05f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors_3[0].motor_data->pid[cascade_inner], 0.006f, 0.0001f, 0.0001f, 3.0f, 9.0f, 0.0f);
    PID_Init(&motors_3[0].motor_data->pid[cascade_outer], 9.7f, 0.0025f, 0.001f, 10.0f, 100.0f, 0.0f);

    // PID_Init(&motors_3[0].motor_data->pid[cascade_inner], 0.006f, 0.0001f, 0.0001f, 3.0f, 9.0f, 0.0f);
    // PID_Init(&motors_3[0].motor_data->pid[cascade_outer], 9.7f, 0.001f, 0.001f, 10.0f, 100.0f, 0.0f);

    // MATLAB离线计算的K矩阵 (默认)
  static const float K[2][4] = {
      {0.5029617493, 0.0215646166,    0.0f,     0.0f    },  // K[0][1] 置零
      {0.0f,      0.0f,   10.0f,    3.2032f  }
  };
    LqrBasic_Init(&gimbal_cmd_send.lqr, K, 1.0f, 1.0f);

    Motor4310_enable_judge(Gimbal_Big_Yaw_4310);//4310电机使能
    if(Gimbal_Big_Yaw_4310[0].motor_data->feedback->tor!=0.0f)
    {
        Motor4310_enable_judge(Gimbal_Pitch_4310);
    }
    
}


/* 遥控器与模式管理 -------------------------------------------------------------- */

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
            Gimbal_Cmd->pitch = -(float)rc_ctrl.rc.ch[3];    // PITCH轴控制（上下俯仰）    
            Gimbal_Cmd->yaw_vel = 0;                        // YAW轴速度设为0
            Gimbal_Cmd->pitch_vel = 0;                      // PITCH轴速度设为0
            Gimbal_Motor_Status(Motor_Enable, Gimbal_Small_Yaw_6020,Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, 1);
            break;
            
        case GIMBAL_ABSOLUTE_ANGLE:             
            Gimbal_Cmd->yaw = -(float)rc_ctrl.rc.ch[2] * yaw_sensitivity;     // YAW轴控制（左右旋转）
            Gimbal_Cmd->pitch = (float)rc_ctrl.rc.ch[3] * pitch_sensitivity;    // PITCH轴控制（上下俯仰）
            Gimbal_Cmd->yaw_vel = 0;                        // YAW轴速度设为0
            Gimbal_Cmd->pitch_vel = 0;                      // PITCH轴速度设为0
            Gimbal_Motor_Status(Motor_Enable, Gimbal_Small_Yaw_6020,Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, 1);
            break;
            
        case GIMBAL_ZERO_FORCE:     
            Gimbal_Cmd->yaw = 0;        // YAW轴目标值为0
            Gimbal_Cmd->pitch = 0;      // PITCH轴目标值为0
            Gimbal_Cmd->yaw_vel = 0;    // YAW轴速度为0
            Gimbal_Cmd->pitch_vel = 0;  // PITCH轴速度为0
            Gimbal_Motor_Status(Motor_Disable, Gimbal_Small_Yaw_6020,Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, 1);
            break;
        
        case GIMBAL_NO_FOLLOW:     
            Gimbal_Cmd->yaw = 0;        // YAW轴目标值为0
            Gimbal_Cmd->pitch = 0;      // PITCH轴目标值为0
            Gimbal_Cmd->yaw_vel = 0;    // YAW轴速度为0
            Gimbal_Cmd->pitch_vel = 0;  // PITCH轴速度为0
            Gimbal_Motor_Status(Motor_Enable, Gimbal_Small_Yaw_6020,Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, 1);
            break;
        case GIMBAL_AUTO:     
            Gimbal_Cmd->yaw = cdc_proto_link.gimbal_cmd.yaw;        // YAW轴目标值为0
            Gimbal_Cmd->pitch = cdc_proto_link.gimbal_cmd.pitch;      // PITCH轴目标值为0
            Gimbal_Cmd->yaw_vel = vision_feedback.yaw_vel;    // YAW轴速度为0
            Gimbal_Cmd->pitch_vel = vision_feedback.pitch_vel;  // PITCH轴速度为0
            Gimbal_Motor_Status(Motor_Enable, Gimbal_Small_Yaw_6020,Gimbal_Big_Yaw_4310,Gimbal_Pitch_4310, 1);
            break;
        default:
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
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break; 
            
        case switch_mid:
            // 拨杆中间：绝对角度模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;
            
        case switch_up:
            // 拨杆向上：导航自瞄模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_AUTO;
            break;    
            
        default:
            break;
    }
    #endif  
}

/**
 * @brief 云台电机状态设置函数
 * @param[in] status 电机状态（使能或失能）
 * @param[in] motors 电机实例数组指针
 * @param[in] motor_count 电机数量
 * @details 根据指定状态批量设置云台电机的使能状态
 */
void Gimbal_Motor_Status(Motor_status status, MotorInstance *motors_1, MotorInstance *motors_2,MotorInstance *motors_3 ,uint8_t motor_count)
{
    switch (status)
    {
        case Motor_Enable:
            // 使能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_enable(motors_1[i].motor_data);
                DM_Motor_enable(motors_2[i].motor_data);
                DM_Motor_enable(motors_3[i].motor_data);
            }
            break;
            
        case Motor_Disable:
            // 失能所有指定的电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_disable(motors_1[i].motor_data);
                DM_Motor_disable(motors_2[i].motor_data);
                DM_Motor_disable(motors_3[i].motor_data);
            }
            break;
            
        default:
            // 默认情况下失能所有电机
            for(int i = 0; i < motor_count; i++)
            {
                DJI_Motor_disable(motors_1[i].motor_data);
                DM_Motor_disable(motors_2[i].motor_data);
                DM_Motor_disable(motors_3[i].motor_data);
            }
            break;
    }
}


/* 控制主函数 -------------------------------------------------------------------- */

/**
 * @brief 云台控制主函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] vision_feedback 视觉反馈数据指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 根据导航标志位选择控制模式（导航或手动）
 */
void Gimbal_control(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw, MotorInstance *motors_pitch,Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, proto_link_t *vision_feedback, INS_t *ins)
{
    
    // 根据导航标志位选择控制模式
    if(Gimbal_Cmd->gimbal_mode == GIMBAL_AUTO )
    {
        if(vision_feedback->decision_cmd.gimbal_permit == 1 && vision_feedback->gimbal_cmd.gimbal_enable == 1)
        {
            // // 导航自瞄模式开启
            Auto_cal(motors_small_yaw, motors_big_yaw,motors_pitch,vision_feedback, ins);
        }
        else
        {
            //视自瞄模式关闭，进行360度连续扫描
            Auto_cal(motors_small_yaw, motors_big_yaw,motors_pitch,vision_feedback, ins);
            // Scan_cal(motors_small_yaw, motors_big_yaw,motors_pitch, ins);
        }
    }
    else if(Gimbal_Cmd->gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        // 跟随模式关闭pitch 96.9 - 34.2
        //limited_cal(motors_small_yaw, motors_big_yaw,motors_pitch, Gimbal_Cmd,180,0.0f,60.0f);
        control_cal(motors_small_yaw, motors_big_yaw,motors_pitch, Gimbal_Cmd,ins);
    }
    else
    {         
    // // 导航模式关闭：执行遥控器手动控制
           // control_cal(motors_small_yaw, motors_big_yaw,motors_pitch, Gimbal_Cmd,ins);
    }
}


/* 控制算法 ---------------------------------------------------------------------- */

/**
 * @brief 自瞄云台运动函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 实现基于遥控器输入的云台手动控制，包含PID控制和机械限位保护
 */
void Auto_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, proto_link_t *vision_feedback, INS_t *ins)
{
    float small_yaw_ecd = 0; //小yaw编码器的数值，转化为角度0-360
    small_yaw_ecd = motors_small_yaw[0].motor_data->feedback->pos / 22.75278f;
    float small_yaw_target = small_yaw_target_ecd;
    while(small_yaw_ecd > 180.0f) 
    {
        small_yaw_ecd -= 360.0f;
    }
    while(small_yaw_ecd < -180.0f) 
    {
        small_yaw_ecd += 360.0f;
    }//归一化到-180-180
    float ecd_error = small_yaw_target - small_yaw_ecd;
    if (ecd_error > 180.0f)
    {
        small_yaw_target -= 360.0f;
        ecd_error -= 360.0f;
    }
    else if (ecd_error < -180.0f)
    {
        small_yaw_target += 360.0f;
        ecd_error += 360.0f;
    }
    if(abs(ecd_error) > ecd_error_dead_zone) 
    {
        PID_CascadeCalc(motors_big_yaw[0].motor_data->pid, small_yaw_target, small_yaw_ecd, motors_small_yaw[0].motor_data->feedback->vel* RPM_TO_RADS);//目标的误差是0
        // 设置YAW轴电机目标位置
        motors_big_yaw[0].motor_data->vel = motors_big_yaw[0].motor_data->pid[cascade_inner].Out;
    }
    else 
    {
        motors_big_yaw[0].motor_data->vel = 0;
    }


    
    // YAW轴控制（偏航轴，左右旋转）
    float yaw_current = ins->Yaw;  // 范围在-180到180
    static float yaw_target = 0;           // YAW目标角度，初始为偏移量
    static float pitch_target = 0;   // PITCH目标角度，初始为偏移量
    static uint8_t yaw_first_run = 1;
    if(yaw_first_run == 1)
    {
        yaw_target = yaw_current;
        yaw_first_run = 0;
    }    
    // 根据输入更新YAW目标角度
    yaw_target = vision_feedback->gimbal_cmd.yaw;
    if(yaw_target == 0.0f)
    {
        yaw_target = yaw_current;
    }

    while(yaw_target > 180.0f) 
    {
        yaw_target -= 360.0f;
    }
    while(yaw_target < -180.0f) 
    {
        yaw_target += 360.0f;
    }
    
    float yaw_error = yaw_target - yaw_current;
    
    while(yaw_error > 180.0f) 
    {
        yaw_error -= 360.0f;
    }
    while(yaw_error < -180.0f) 
    {
        yaw_error += 360.0f;
    }
    // 执行YAW轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（角速度）
    PID_CascadeCalc(motors_small_yaw[0].motor_data->pid, yaw_error, 0, INS.Gyro[Zt] * hudu);//目标的误差是0
    motors_small_yaw[0].motor_data->target_current = motors_small_yaw[0].motor_data->pid[cascade_inner].Out;





    // PITCH轴控制（俯仰轴，上下运动）
    // 定义机械限位范围
    const float pitch_max_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_min_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    // 使用陀螺仪PITCH角度作为反馈值
    //float pitch_current = pitch_angle;//ins->Pitch;           // 获取当前PITCH角度
    float pitch_current = ins->Pitch;  
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run == 1)
    {
        pitch_target = pitch_current;
        pitch_first_run = 0;
    } 
    // 根据遥控器输入更新PITCH目标角度
    pitch_target = vision_feedback->gimbal_cmd.pitch;
    if(pitch_target == 0.0f)
    {
        pitch_target = pitch_current;
    }  
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_min_limit) 
    {
        // 超过下限则限制在下限值
        pitch_target = pitch_min_limit;
    }
    else if (pitch_target < pitch_max_limit)
    {
        // 超过上限则限制在上限值0
        pitch_target = pitch_max_limit;
    }
 
    float target_tor = 0.0f;
    // 执行PITCH轴串级PID外环位置环
    PID_CascadeCalc(motors_pitch[0].motor_data->pid, pitch_target, pitch_current, ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标位置
    target_tor = motors_pitch[0].motor_data->pid[cascade_inner].Out; 
    // 设置YAW轴电机目标位置
    motors_pitch[0].motor_data->tor = target_tor;
    motors_pitch[0].motor_data->kp = 0;
    motors_pitch[0].motor_data->kd = 0; 
}

/**
 * @brief 控制云台运动函数
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 实现基于遥控器输入的云台手动控制，包含PID控制和机械限位保护
 */
void control_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd, INS_t *ins)
{
    float small_yaw_ecd = 0; //小yaw编码器的数值，转化为角度0-360
    small_yaw_ecd = motors_small_yaw[0].motor_data->feedback->pos / 22.75278f;
    float small_yaw_target = small_yaw_target_ecd;
    while(small_yaw_ecd > 180.0f) 
    {
        small_yaw_ecd -= 360.0f;
    }
    while(small_yaw_ecd < -180.0f) 
    {
        small_yaw_ecd += 360.0f;
    }//归一化到-180-180
    float ecd_error = small_yaw_target - small_yaw_ecd;
    if (ecd_error > 180.0f)
    {
        small_yaw_target -= 360.0f;
        ecd_error -= 360.0f;
    }
    else if (ecd_error < -180.0f)
    {
        small_yaw_target += 360.0f;
        ecd_error += 360.0f;
    }
    if(abs(ecd_error) > ecd_error_dead_zone ) 
    {
        PID_CascadeCalc(motors_big_yaw[0].motor_data->pid, small_yaw_target, small_yaw_ecd, motors_small_yaw[0].motor_data->feedback->vel* RPM_TO_RADS);//目标的误差是0
        // 设置YAW轴电机目标位置
        motors_big_yaw[0].motor_data->vel = motors_big_yaw[0].motor_data->pid[cascade_inner].Out + chassis_cmd_send.w * w_out_comp ;
    }
    else 
    {
        motors_big_yaw[0].motor_data->vel= 0;
    }


    
    // YAW轴控制（偏航轴，左右旋转）
    float yaw_current = ins->Yaw;  // 范围在-180到180
    static float yaw_target = 0;           // YAW目标角度，初始为偏移量
    static float pitch_target = 0;   // PITCH目标角度，初始为偏移量
    static uint8_t yaw_first_run = 1;
    if(yaw_first_run == 1)
    {
        yaw_target = yaw_current;
        yaw_first_run = 0;
    }    
    // 根据遥控器输入更新YAW目标角度
    yaw_target += Gimbal_Cmd->yaw;
    while(yaw_target > 180.0f) 
    {
        yaw_target -= 360.0f;
    }
    while(yaw_target < -180.0f) 
    {
        yaw_target += 360.0f;
    }
    
    float yaw_error = yaw_target - yaw_current;
    
    while(yaw_error > 180.0f) 
    {
        yaw_error -= 360.0f;
    }
    while(yaw_error < -180.0f) 
    {
        yaw_error += 360.0f;
    }
    // 执行YAW轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（角速度）
    PID_CascadeCalc(motors_small_yaw[0].motor_data->pid, yaw_error, 0, INS.Gyro[Zt] * hudu);//目标的误差是0
    motors_small_yaw[0].motor_data->target_current = motors_small_yaw[0].motor_data->pid[cascade_inner].Out;





    // PITCH轴控制（俯仰轴，上下运动）
    // 定义机械限位范围
    const float pitch_max_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_min_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    // 使用陀螺仪PITCH角度作为反馈值          // 获取当前PITCH角度
    float pitch_current = ins->Pitch;  
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run == 1)
    {
        pitch_target = pitch_current;
        pitch_first_run = 0;
    } 
    // 根据遥控器输入更新PITCH目标角度
    pitch_target += Gimbal_Cmd->pitch;  
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_min_limit) 
    {
        // 超过下限则限制在下限值
        pitch_target = pitch_min_limit;
    }
    else if (pitch_target < pitch_max_limit)
    {
        // 超过上限则限制在上限值0
        pitch_target = pitch_max_limit;
    }

    float target_tor = 0.0f;
    // 执行PITCH轴串级PID外环位置环
    PID_CascadeCalc(motors_pitch[0].motor_data->pid, pitch_target, pitch_current, ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标位置
    target_tor = motors_pitch[0].motor_data->pid[cascade_inner].Out; 
    // 设置YAW轴电机目标位置
    motors_pitch[0].motor_data->tor =target_tor + Gravity_compensation;
    motors_pitch[0].motor_data->kp = 0;
    motors_pitch[0].motor_data->kd = 0;  
}


/**
 * @brief 限定云台运动函数(主要还是测试用)
 * @param[in] motors 电机实例数组指针
 * @param[in,out] Gimbal_Cmd 云台控制命令结构体指针
 * @param[in] ins 惯性导航系统数据指针
 * @details 实现基于遥控器输入的云台手动控制，包含PID控制和机械限位保护
 */
void limited_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,MotorInstance *motors_pitch, Gimbal_Ctrl_Cmd_s *Gimbal_Cmd , float small_yaw_ecd_target , float big_yaw_ecd_target,float pitch_ecd_target)
{
    float small_yaw_ecd = 0; //小yaw编码器的数值，转化为角度0-360
    small_yaw_ecd =motors_small_yaw[0].motor_data->feedback->pos / 22.75278f;
    float small_yaw_target =small_yaw_ecd_target;
    while(small_yaw_ecd > 180.0f) 
    {
        small_yaw_ecd -= 360.0f;
    }
    while(small_yaw_ecd < -180.0f) 
    {
        small_yaw_ecd += 360.0f;
    }//归一化到-180-180
    float ecd_error = small_yaw_target - small_yaw_ecd;
    if (ecd_error > 180.0f)
    {
        small_yaw_target -= 360.0f;
        ecd_error -= 360.0f;
    }
    else if (ecd_error < -180.0f)
    {
        small_yaw_target += 360.0f;
        ecd_error += 360.0f;
    }
    PID_CascadeCalc(motors_small_yaw[0].motor_data->pid, small_yaw_target, small_yaw_ecd, motors_small_yaw[0].motor_data->feedback->vel* RPM_TO_RADS);//目标的误差是0
    // 设置YAW轴电机目标位置
    motors_small_yaw[0].motor_data->target_current = motors_small_yaw[0].motor_data->pid[cascade_inner].Out;
    LQR_Controller_Loop(motors_small_yaw,small_yaw_target/hudu,0,motors_pitch,0,0,Gimbal_Cmd);//注意这里的单位是rad



    
    // 大YAW轴控制（偏航轴，左右旋转）
    float big_yaw_ecd = 0; //小yaw编码器的数值，转化为角度0-360
    big_yaw_ecd = motors_big_yaw[0].motor_data->feedback->pos * hudu;
    float big_yaw_target = big_yaw_ecd_target;
    while(big_yaw_ecd > 180.0f) 
    {
        big_yaw_ecd -= 360.0f;
    }
    while(big_yaw_ecd < -180.0f) 
    {
        big_yaw_ecd += 360.0f;
    }//归一化到-180-180
    float ecd_error_big = big_yaw_target - big_yaw_ecd;
    if (ecd_error_big > 180.0f)
    {
        big_yaw_target -= 360.0f;
        ecd_error_big -= 360.0f;
    }
    else if (ecd_error_big < -180.0f)
    {
        big_yaw_target += 360.0f;
        ecd_error_big += 360.0f;
    }
    PID_CascadeCalc(motors_big_yaw[0].motor_data->pid, big_yaw_target, big_yaw_ecd, motors_big_yaw[0].motor_data->feedback->vel* RPM_TO_RADS);//目标的误差是0
    // 设置YAW轴电机目标位置
    motors_big_yaw[0].motor_data->vel = motors_big_yaw[0].motor_data->pid[cascade_inner].Out;





    // PITCH轴控制（俯仰轴，上下运动）
    float pitch_target = pitch_ecd_target;
    float pitch_current = motors_pitch[0].motor_data->feedback->pos*hudu;  


    float target_tor = 0.0f;
    // 执行PITCH轴串级PID外环位置环
    PID_CascadeCalc(motors_pitch[0].motor_data->pid, pitch_target, pitch_current, motors_pitch[0].motor_data->feedback->vel* RPM_TO_RADS);
    // 设置PITCH轴电机目标位置
    target_tor = motors_pitch[0].motor_data->pid[cascade_inner].Out; 
    // 设置YAW轴电机目标位置
    motors_pitch[0].motor_data->tor = target_tor;
    motors_pitch[0].motor_data->kp = 0;
    motors_pitch[0].motor_data->kd = 0;
}

/**
 * @brief 云台360°连续扫描函数
 * @param[in] motors_small_yaw 小YAW轴电机（6020）
 * @param[in] motors_big_yaw   大YAW轴电机（4310）
 * @param[in] motors_pitch     PITCH轴电机（4310）
 * @param[in] ins              惯导数据
 * @details yaw_target 持续累加，通过将 yaw_current 展开到连续域
 *          使误差始终为正向小量，实现单向连续360°旋转
 */
void Scan_cal(MotorInstance *motors_small_yaw, MotorInstance *motors_big_yaw,
              MotorInstance *motors_pitch, INS_t *ins)
{
    float dt = 0.002f;//对应两毫秒
    float small_yaw_ecd = 0; //小yaw编码器的数值，转化为角度0-360
    small_yaw_ecd = motors_small_yaw[0].motor_data->feedback->pos / 22.75278f;
    float small_yaw_target = small_yaw_target_ecd;
    while(small_yaw_ecd > 180.0f) 
    {
        small_yaw_ecd -= 360.0f;
    }
    while(small_yaw_ecd < -180.0f) 
    {
        small_yaw_ecd += 360.0f;
    }//归一化到-180-180
    float ecd_error = small_yaw_target - small_yaw_ecd;
    if (ecd_error > 180.0f)
    {
        small_yaw_target -= 360.0f;
        ecd_error -= 360.0f;
    }
    else if (ecd_error < -180.0f)
    {
        small_yaw_target += 360.0f;
        ecd_error += 360.0f;
    }
    if(abs(ecd_error) > ecd_error_dead_zone ) 
    {
        PID_CascadeCalc(motors_big_yaw[0].motor_data->pid, small_yaw_target, small_yaw_ecd, motors_small_yaw[0].motor_data->feedback->vel* RPM_TO_RADS);//目标的误差是0
        // 设置YAW轴电机目标位置
        motors_big_yaw[0].motor_data->vel = motors_big_yaw[0].motor_data->pid[cascade_inner].Out+ chassis_cmd_send.w * w_out_comp;
    }
    else 
    {
        motors_big_yaw[0].motor_data->vel= 0;
    }


    
    // YAW轴控制（偏航轴，左右旋转），持续360°旋转
    float yaw_current = ins->Yaw;  // 范围在-180到180
    static float yaw_target = 0;
    static float pitch_target = 0;   // PITCH目标角度，初始为偏移量
    static uint8_t yaw_first_run = 1;
    if(yaw_first_run == 1)
    {
        yaw_target = yaw_current;
        yaw_first_run = 0;
    }
    // 增量更新YAW目标角度，持续朝一个方向旋转
    yaw_target += SCAN_SPEED * dt;
    // 将目标角度维持在[-180, 180]范围内
    if(yaw_target > 180.0f)
    {
        yaw_target -= 360.0f;
    }
    if(yaw_target < -180.0f)
    {
        yaw_target += 360.0f;
    }
    
    float yaw_error = yaw_target - yaw_current;
    
    while(yaw_error > 180.0f) 
    {
        yaw_error -= 360.0f;
    }
    while(yaw_error < -180.0f) 
    {
        yaw_error += 360.0f;
    }
    // 执行YAW轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（角速度）
    PID_CascadeCalc(motors_small_yaw[0].motor_data->pid, yaw_error, 0, INS.Gyro[Zt] * hudu);//目标的误差是0
    motors_small_yaw[0].motor_data->target_current = motors_small_yaw[0].motor_data->pid[cascade_inner].Out;





    // PITCH轴控制（俯仰轴，上下运动）
    static Signal sig_pitch;
    static uint8_t sig_pitch_init = 1;
    if(sig_pitch_init == 1)
    {
        Signal_Init(&sig_pitch, 20.0f, 0.5f, 0.0f, 0.0f);
        sig_pitch_init = 0;
    }

    // 定义机械限位范围
    const float pitch_max_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_min_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    // 使用陀螺仪PITCH角度作为反馈值          // 获取当前PITCH角度
    float pitch_current = ins->Pitch;  
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run == 1)
    {
        pitch_target = pitch_current;
        pitch_first_run = 0;
    } 
    // 根据遥控器输入更新PITCH目标角度
    pitch_target = Signal_Sin_Gen(&sig_pitch,dt);  
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_min_limit) 
    {
        // 超过下限则限制在下限值
        pitch_target = pitch_min_limit;
    }
    else if (pitch_target < pitch_max_limit)
    {
        // 超过上限则限制在上限值0
        pitch_target = pitch_max_limit;
    }

    float target_tor = 0.0f;
    // 执行PITCH轴串级PID外环位置环
    PID_CascadeCalc(motors_pitch[0].motor_data->pid, pitch_target, pitch_current, ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标位置
    target_tor = motors_pitch[0].motor_data->pid[cascade_inner].Out; 
    // 设置YAW轴电机目标位置
    motors_pitch[0].motor_data->tor = target_tor + Gravity_compensation;
    motors_pitch[0].motor_data->kp = 0;
    motors_pitch[0].motor_data->kd = 0;
}

/**
 * @brief LQR控制器循环计算函数
 * @param Motor_Small_Yaw 小YAW轴电机实例指针
 * @param small_yaw_target 小YAW轴目标角度 (rad)
 * @param small_yaw_vel_target 小YAW轴目标角速度 (rad/s)
 * @param Motor_Pitch PITCH轴电机实例指针
 * @param pitch_target PITCH轴目标角度 (rad)
 * @param pitch_vel_target PITCH轴目标角速度 (rad/s)
 * @param gimbal_ctrl 云台控制命令结构体指针
 * @details 计算小YAW轴和PITCH轴的LQR控制量，输出电机转矩
 */
void LQR_Controller_Loop(MotorInstance* Motor_Small_Yaw ,float small_yaw_target,float small_yaw_vel_target,MotorInstance* Motor_Pitch ,float pitch_target,float pitch_vel_target , Gimbal_Ctrl_Cmd_s* gimbal_ctrl)
{
    //对电机反馈角速度的一阶滤波
    static float phi_dot_lpf = 0.0f;
    float phi_dot_raw = Motor_Small_Yaw->motor_data->feedback->vel * RPM_TO_RADS;//转换为弧度
    phi_dot_lpf += 0.15f * (phi_dot_raw - phi_dot_lpf);
    float small_yaw_error = small_yaw_target - Motor_Small_Yaw->motor_data->feedback->pos* (2.0f * 3.141592653589793f / 8191.0f);//rad
    if(small_yaw_error > 3.141592653589793f)
    {
        small_yaw_error -= 2.0f * 3.141592653589793f;
    }
    if(small_yaw_error < -3.141592653589793f)
    {
        small_yaw_error += 2.0f * 3.141592653589793f;
    }
    float pitch_error = pitch_target - Motor_Pitch->motor_data->feedback->pos;
    if(pitch_error > 3.141592653589793f)
    {
        pitch_error -= 2.0f * 3.141592653589793f;
    }
    if(pitch_error < -3.141592653589793f)
    {
        pitch_error += 2.0f * 3.141592653589793f;
    }      
    LqrBasic_Calc(&gimbal_ctrl->lqr,small_yaw_error , small_yaw_vel_target-phi_dot_lpf, pitch_error, pitch_vel_target- Motor_Pitch->motor_data->feedback->vel);//单位都是rad,rad/s
    Motor_Small_Yaw->motor_data->target_current =gimbal_ctrl->lqr.tor_phi /0.741f *16384.0f/3.0f;//最终的输出需要加负号,tor_phi单位是NM
    Motor_Pitch->motor_data->tor = gimbal_ctrl->lqr.tor_theta;
    Motor_Pitch->motor_data->kp = 0;
    Motor_Pitch->motor_data->kd = 0;  
}

/**
 * @brief 底盘云台相对角度计算函数
 * @param Motor_Big_Yaw 大YAW轴电机实例指针
 * @details 计算大YAW轴和底盘之间的相对角度输出单位为rad,最后用于发送给上位机
 */
void Big_Yaw_Chassis_Relative_Angle_Update(MotorInstance* Motor_Big_Yaw)
{
    float Big_Yaw_Chassis_Relative_Angle = Motor_Big_Yaw->motor_data->feedback->pos * hudu - big_yaw_chassis_ecd;
    if (Big_Yaw_Chassis_Relative_Angle > 180.0f)
    {
        Big_Yaw_Chassis_Relative_Angle -= 360.0f;
    }
    else if (Big_Yaw_Chassis_Relative_Angle < -180.0f)
    {
        Big_Yaw_Chassis_Relative_Angle += 360.0f;
    }

    host_report.yaw_odom = -Big_Yaw_Chassis_Relative_Angle / hudu;//规定逆时针为正方向，单位为rad
}
#endif