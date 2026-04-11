#include "gimbal.h"
#ifdef COMPILE_GIMBAL
/**
 * @brief 云台控制命令结构体
 * @details 用于存储云台控制的各种参数和命令
 */
Gimbal_Ctrl_Cmd_s gimbal_cmd_control;
/**
 * @brief 云台测量数据结构体
 * @details 用于存储云台当前的状态测量数据，传输给视觉系统
 */
Gimbal_measure gimbal_measure_data;

static void gimbal_set_mode(void);

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
float pitch_angle = 0.0f;

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
        // 设置云台电机模式
        gimbal_set_mode();
        // 处理遥控器输入数据，映射到云台控制命令
        RemoteControlGimbal(&gimbal_cmd_send);
        // 更新云台控制模式（根据遥控器拨杆状态）
        Gimbal_mode_update(&gimbal_cmd_send);
        //PID数据切换
        Gimbal_PID_Change(Gimbal_6020);
        // 检测模式切换，清除PID积分器以避免突变
        if(last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE)
        {
            Gimbal_PIDClear(Gimbal_6020);
        }
        last_mode = gimbal_cmd_send.gimbal_mode;
        // 根据当前云台模式执行相应的控制逻辑
        switch (gimbal_cmd_send.gimbal_mode)
        {
            case GIMBAL_ZERO_FORCE:
                // 零电流模式：关闭电机使能，电机不输出力矩
                Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
                break;
            
            case GIMBAL_NO_FOLLOW:
                // 零电流模式：关闭电机使能，电机不输出力矩
                Gimbal_Motor_Status(Motor_Disable, Gimbal_6020, 2);
                break;  

            case GIMBAL_NORMAL:
                // 正常模式：使能电机并执行回零点操作
                Gimbal_Motor_Status(Motor_Enable, Gimbal_6020, 2);
                Gimbal_control(Gimbal_6020, &gimbal_cmd_send, &vision_feedback, &INS);
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
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 180.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 8.0f, 0.0f, 0.0f, 10.0f, 190.0f, 20.0f);
    
    // PITCH轴（俯仰轴）6020电机PID初始化
    PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 45.0f, 0.0f, 0.0f, 300.0f, 16000.0f, 0.0f);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 38.0f, 0.0f, 0.0f, 0.0f, 165.0f, 0.0f);
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
            Gimbal_Cmd->gimbal_mode = GIMBAL_NORMAL;
            break;
            
        case switch_up:
            // 拨杆向上：绝对角度模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;    
            
        default:
            break;
    }
    #endif
    #ifdef FS_REMOTE
    // 根据左拨杆位置设置云台模式
    switch (right_switch)
    {
        case switch_up:
            // 拨杆向下：零电流模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ZERO_FORCE;
            break; 
            
        case switch_mid:
            // 拨杆中间：正常模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_NORMAL;
            break;
            
        case switch_down:
            // 拨杆向上：绝对角度模式
            Gimbal_Cmd->gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            break;    
            
        default:
            break;
    }
    #endif
    #ifdef VT_03_REMOTE
        static uint8_t last_switch_state = 0xFF;
        static gimbal_mode_e current_gimbal_mode = GIMBAL_NORMAL;  // 用 static 保存模式，初始为 GIMBAL_NORMAL

        Gimbal_Cmd->gimbal_mode = current_gimbal_mode;  // 使用保存的模式

        // R键循环切换模式（边沿检测）
        if (rc_ctrl.key_rising_edge & (1 << 8))
        {
            if (current_gimbal_mode == GIMBAL_NORMAL)
            {
                current_gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
            }
            else if (current_gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)
            {
                current_gimbal_mode = GIMBAL_NORMAL;
            }
            Gimbal_Cmd->gimbal_mode = current_gimbal_mode;
        }
        // 开关切换模式（边沿检测，只有开关位置变化时才切换）
        else if ((left_switch != last_switch_state) && (left_switch != switch_down))
        {
            switch (left_switch)
            {
                case switch_down:
                    current_gimbal_mode = GIMBAL_NO_FOLLOW;
                    break;
                case switch_mid:
                    current_gimbal_mode = GIMBAL_NORMAL;
                    break;
                case switch_up:
                    current_gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;
                    break;
                default:
                    current_gimbal_mode = GIMBAL_ZERO_FORCE;
                    break;
            }
            Gimbal_Cmd->gimbal_mode = current_gimbal_mode;
            last_switch_state = left_switch;
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
            // 键盘有操作时用键盘，否则用摇杆
            if (rc_ctrl.mouse_x != 0 || rc_ctrl.mouse_y != 0)
            {
                Gimbal_Cmd->yaw = -rc_ctrl.mouse_x * 2.2;
                Gimbal_Cmd->pitch = -rc_ctrl.mouse_y * 2.2;
            }
            else
            {
                // 正常模式：直接映射遥控器摇杆数据
                Gimbal_Cmd->yaw = -(float)rc_ctrl.rc.ch[2];     // YAW轴控制（左右旋转）
                Gimbal_Cmd->pitch = (float)rc_ctrl.rc.ch[3];    // PITCH轴控制（上下俯仰）
            }
            Gimbal_Cmd->yaw_vel = 0;                        // YAW轴速度设为0
            Gimbal_Cmd->pitch_vel = 0;                      // PITCH轴速度设为0
            break;
            
        case GIMBAL_ABSOLUTE_ANGLE:             
            // 绝对角度模式：同样映射遥控器摇杆数据
            // 键盘有操作时用键盘，否则用摇杆
            if (rc_ctrl.mouse_x != 0 || rc_ctrl.mouse_y != 0)
            {
                Gimbal_Cmd->yaw = -rc_ctrl.mouse_x * 2.2;
                Gimbal_Cmd->pitch = -rc_ctrl.mouse_y * 2.2;
            }
            else
            {
                // 正常模式：直接映射遥控器摇杆数据
                Gimbal_Cmd->yaw = -(float)rc_ctrl.rc.ch[2];     // YAW轴控制（左右旋转）
                Gimbal_Cmd->pitch = (float)rc_ctrl.rc.ch[3];    // PITCH轴控制（上下俯仰）
            }
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
    motors->motor_control(motors);
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

    #ifdef FS_REMOTE
    switch (left_switch)
    {
    case switch_up:
        // 侧拨杆关闭：关闭自瞄模式
        auto_aim_flag = 0;
        break;
    case switch_down:
        // 侧拨杆打开：开启自瞄模式
        auto_aim_flag = 1;
        break;
    default:
        // 侧拨杆关闭：关闭自瞄模式
        auto_aim_flag = 0;
        break;
    }
    #endif

    #ifdef VT_03_REMOTE
        // 鼠标右键切换自瞄模式（边沿检测）
        static uint8_t mouse_right_last = 0;
        uint8_t mouse_right_now = rc_ctrl.mouse_right;
        
        if (mouse_right_now && !mouse_right_last)  // 右键按下瞬间
        {
            auto_aim_flag = !auto_aim_flag;  // 切换状态
        }
        mouse_right_last = mouse_right_now;
    #endif
    
    // 根据自瞄标志位选择控制模式
    switch (auto_aim_flag)
    {
        case AUTO_AIM_ON:
            if(vision_feedback->mode == 1 || vision_feedback->mode == 2)
            {
                // 自瞄模式开启：执行自动瞄准计算
                Auto_aiming_cal(motors, Gimbal_Cmd, vision_feedback, ins);
            }
            break;           
        case AUTO_AIM_OFF:
            // 自瞄模式关闭：执行遥控器手动控制
            remote_control(motors, Gimbal_Cmd, ins);
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
    //static float yaw_target = 0;           // YAW目标角度，初始为偏移量
    static uint8_t yaw_first_run = 1;
    if(yaw_first_run == 1)
    {
        yaw_target = yaw_current;
        yaw_first_run = 0;
    }    
    // 根据遥控器输入更新YAW目标角度
    yaw_target += Gimbal_Cmd->yaw * yaw_sensitivity;
    while(yaw_target > 180.0f) {
        yaw_target -= 360.0f;
    }
    while(yaw_target < -180.0f) {
        yaw_target += 360.0f;
    }
    
    float yaw_error = yaw_target - yaw_current;
    
    while(yaw_error > 180.0f) {
        yaw_error -= 360.0f;
    }
    while(yaw_error < -180.0f) {
        yaw_error += 360.0f;
    }
    // 执行YAW轴串级PID控制计算
    // 参数：PID控制器、目标角度、当前角度、前馈值（角速度）
    PID_CascadeCalc(motors[0].motor_data->pid, yaw_error, 0, ins->Gyro[Zt] * hudu);
    
    // 设置YAW轴电机目标位置
    motors[0].motor_data->target_current = motors[0].motor_data->pid[cascade_inner].Out;
    
    // PITCH轴控制（俯仰轴，上下运动）
    // 定义机械限位范围
    const float pitch_up_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_down_limit = pitch_down_max;    // PITCH轴最小值（向下限位）

    // 使用陀螺仪PITCH角度作为反馈值
    //float pitch_current = pitch_angle;//陀螺仪pitch飘用这个
    float pitch_current = ins->Pitch;  
    //static float pitch_target = 0;   // PITCH目标角度，初始为偏移量
    static uint8_t pitch_first_run = 1;
    if(pitch_first_run == 1)
    {
        pitch_target = pitch_current;
        pitch_first_run = 0;
    } 
    // 根据遥控器输入更新PITCH目标角度
    pitch_target += Gimbal_Cmd->pitch * pitch_sensitivity;  
    //pitch_target = 0;
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
    PID_CascadeCalc(motors[1].motor_data->pid, pitch_target, pitch_current, ins->Gyro[Xt] * hudu);
    // 设置PITCH轴电机目标位置
    motors[1].motor_data->target_current = motors[1].motor_data->pid[cascade_inner].Out + Gravity_compensation(ins->Pitch);
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
    //自瞄控制
    static uint8_t first_run = 1;
    
    // 陀螺仪绝对角度闭环
    float yaw_current = ins->Yaw;  // 当前陀螺仪YAW角度（-180到180）
    float pitch_current = ins->Pitch;  // 获取当前PITCH角度
    // ========== PITCH轴控制 ==========
    // 定义机械限位范围
    const float pitch_up_limit = pitch_up_max;    // PITCH轴最大值（向上限位）
    const float pitch_down_limit = pitch_down_max;    // PITCH轴最小值（向下限位）
  
    // 确定目标角度
    //float yaw_target;
    //float pitch_target;
    
    // 检测视觉数据是否有效
    uint8_t vision_valid = 1;
    
    // 判断条件：如果yaw和pitch都是0，认为是无效数据
    // 注意：这里假设0,0表示视觉丢失
    if (vision_feedback->yaw == 0.0f && vision_feedback->pitch == 0.0f)
    {
        vision_valid = 0;
    }  
   
    // 首次运行时，记录当前角度作为目标
    if (first_run)
    {
        if (vision_valid)
        {
            yaw_target_last = vision_feedback->yaw * hudu;
            pitch_target_last = vision_feedback->pitch * hudu;
        }
        else
        {
            // 首次运行且视觉无效，保持当前位置
            yaw_target_last = ins->Yaw;
            pitch_target_last = ins->Pitch;
        }
        yaw_target = yaw_target_last;
        pitch_target = pitch_target_last;
        first_run = 0;
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
        yaw_target = yaw_target_last;
        pitch_target = pitch_target_last;
    }
    
    // 计算控制误差（目标 - 当前） 
    float yaw_error = yaw_target - yaw_current;

    // 将误差标准化到 -180 到 180 范围（就近原则）
    while(yaw_error > 180.0f)
    {
        yaw_error -= 360.0f;
    }
    while(yaw_error < -180.0f)
    {
        yaw_error += 360.0f;
    }
    
    // 执行串级PID控制
    PID_CascadeCalc(motors[0].motor_data->pid, yaw_error, 0, ins->Gyro[Zt] * hudu);
    motors[0].motor_data->target_current = motors[0].motor_data->pid[cascade_inner].Out;
   
    // 机械限位保护：限制目标角度在允许范围内
    if (pitch_target > pitch_down_limit) 
    {
        pitch_target = pitch_down_limit;
    }
    else if (pitch_target < pitch_up_limit)
    {
        pitch_target = pitch_up_limit;
    }
    
    // 执行PITCH轴串级PID控制计算
    PID_CascadeCalc(motors[1].motor_data->pid, pitch_target, pitch_current, ins->Gyro[Xt] * hudu);
    
    // 设置PITCH轴电机目标位置
    motors[1].motor_data->target_current = motors[1].motor_data->pid[cascade_inner].Out + Gravity_compensation(ins->Pitch);
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
            yaw_target_last = 0;
            pitch_target_last = 0;
            PID_Clear(motors[0].motor_data->pid);
            PID_Clear(motors[1].motor_data->pid);
            // YAW轴（偏航轴）6020电机PID初始化
            PID_Init(&motors[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_inner], 215.0f, 0.0f, 0.0f, 10.0f, 11500.0f, 250.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_outer], 14.5f, 0.0f, 0.0f, 10.0f, 160.0f, 100.0f);       
            // PITCH轴（俯仰轴）6020电机PID初始化
            PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_inner], 45.0f, 0.0f, 0.0f, 300.0f, 16000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_outer], 38.0f, 0.0f, 0.0f, 0.0f, 165.0f, 0.0f);

        }
        else
        {
            //视觉上次变量清零：   
            yaw_target_last = 0;
            pitch_target_last = 0;
            PID_Clear(motors[0].motor_data->pid);
            PID_Clear(motors[1].motor_data->pid);
            // YAW轴（偏航轴）6020电机PID初始化
            PID_Init(&motors[0].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_inner], 180.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.0f);
            PID_Init(&motors[0].motor_data->pid[cascade_outer], 8.0f, 0.0f, 0.0f, 10.0f, 190.0f, 20.0f);

            // PITCH轴（俯仰轴）6020电机PID初始化
            PID_Init(&motors[1].motor_data->pid[single_loop], 1.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_inner], 45.0f, 0.0f, 0.0f, 300.0f, 16000.0f, 0.0f);
            PID_Init(&motors[1].motor_data->pid[cascade_outer], 38.0f, 0.0f, 0.0f, 0.0f, 165.0f, 0.0f);
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

//pitch轴重力补偿(输出电流)
float Gravity_compensation(float pitch)
{
    float current = 0;
    current = 0.2465 * pitch * pitch + 39.274 * pitch - 5589.9;
    return current;
}

#endif // COMPILE_GIMBAL


static void gimbal_set_mode(void)
{
  gimbal_behaviour_mode_set(&gimbal_cmd_control);
}
