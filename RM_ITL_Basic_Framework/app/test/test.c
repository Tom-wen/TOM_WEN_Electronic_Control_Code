#include "test.h"
#ifdef COMPILE_TEST
//底盘电机发送数据结构体实例
test_Ctrl_Cmd_s test_cmd_send;
Signal signal_test;
LqrBasic lqr;
/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 * @details 这是底盘控制的主任务函数，负责根据不同的模式控制底盘运动
 */
void test_task(void *argument)
{
    static test_mode_e last_mode = test_ZERO_FORCE;
    test_Init(test_3508, &test_cmd_send);
    DM_Motor_Init(test_4310);                       // DM系列电机初始化
    Signal_Init(&signal_test, 1.5f, 0.5f, 0.0f, 0.0f);
    Upper_Computer_Init(&test_3508[0].motor_data->target_position);
    Upper_Computer_Init(&test_3508[0].motor_data->feedback->vel);
    //RS_Motor_Init(test_EL05, rs_pos_mode);
    //test_EL05[1].motor_data->target_position = 3.6;
    //FT_TorqueEnable(2, 1);       // 先开力矩
    //GQ_Motor_Init(test_3508);
      // 使用
    for(;;)
    {
        test_mode_update(&test_cmd_send);
        // 检测模式切换，清除PID积分器以避免突变
        if(last_mode == test_ZERO_FORCE && test_cmd_send.test_mode != test_ZERO_FORCE)
        {
            test_PIDClear(test_3508);
        }
        last_mode = test_cmd_send.test_mode;
        switch (test_cmd_send.test_mode)
        {
            case test_ZERO_FORCE:
                // 零力模式：关闭所有电机输出
                test_Motor_Status(Motor_Disable, test_3508, 2);
                test_Motor_Status(Motor_Enable, test_4310, 3);
                test_Motor_Status(Motor_Enable, test_EL05, 2);
                break;
            case test_NO_FOLLOW:
                // 无跟随模式：底盘独立运动，不跟随云台
                //test_Motor_Status(Motor_Enable, test_3508, 2);
                test_Motor_Status(Motor_Enable, test_EL05, 2);
                break;
            case test_FOLLOW_GIMBAL_YAW:
                // 跟随云台偏航角模式：底盘运动时会自动补偿云台偏航角
                test_Motor_Status(Motor_Enable, test_3508, 2);
                test_Motor_Status(Motor_Enable, test_EL05, 2);
                break;
            case test_ROTATE:
                // 自旋模式：底盘原地旋转
                test_Motor_Status(Motor_Enable, test_3508, 2);
                break;                            
        }
        //上位机调PID用
        //PID_Test_Init(PID_K, test_3508);
        //3508测试
        // test_3508[0].motor_data->target_velocity = 800;//rc_ctrl.rc.ch[1]*8;
        
        float yaw_target = 3.14f + Signal_Sin_Gen(&signal_test,0.002f);
        LqrBasic_Calc(&lqr,-yaw_target + test_3508[0].motor_data->feedback->pos * (2.0f * 3.141592653589793f / 8191.0f), test_3508[0].motor_data->feedback->vel*0.10472, 0, 0);
        test_3508[0].motor_data->target_current =lqr.tor_phi /0.741f *16384.0f/3.0f;
        //test_3508[0].motor_data->target_current =2000.0f; //Signal_Sin_Gen(&signal_test,0.002);
        // test_3508[0].motor_data->target_current = 3;
        // test_3508[1].motor_data->target_velocity = 300;
        // test_3508[1].motor_data->target_position = rc_ctrl.rc.ch[1] * 0.005;
        // test_3508[1].motor_data->target_current = 3;
        //灵足测试
        test_EL05[0].motor_data->target_velocity = 0;
        test_EL05[0].motor_data->target_position = rc_ctrl.rc.ch[1] * 0.005;
        test_EL05[0].motor_data->target_current = 0;
        test_EL05[1].motor_data->target_velocity = 2;
        test_EL05[1].motor_data->target_position += rc_ctrl.rc.ch[1] * 0.002;
        if(test_EL05[1].motor_data->target_position >= 3.7)
        {
            test_EL05[1].motor_data->target_position = 3.7;
        }
        else if(test_EL05[1].motor_data->target_position <= 3.0)
        {
            test_EL05[1].motor_data->target_position = 3.0;
        }
        test_EL05[1].motor_data->target_current = 3;
        //达妙电机测试
        test_4310[0].motor_data->vel = 2;
        test_4310[1].motor_data->vel = 2;
        test_4310[2].motor_data->vel = 2;
        //高擎测试
        // test_3508[0].motor_data->target_velocity = 1500;
        // test_3508[0].motor_data->target_position = rc_ctrl.rc.ch[1] * 8 - 2200;
        // test_3508[0].motor_data->target_current = 200;
        // test_3508[1].motor_data->target_velocity = 1500;
        // test_3508[1].motor_data->target_position = 5000;     
        // test_3508[1].motor_data->target_current = 200;
        // if(KEY_Z){
        // test_3508[0].motor_data->target_current = 3; 
        // test_3508[0].motor_data->target_velocity = 0;
        // test_3508[0].motor_data->target_position += 0.01;
        // }
        // else if(KEY_X){
        // test_3508[0].motor_data->target_current = 3; 
        // test_3508[0].motor_data->target_velocity = 0;
        // test_3508[0].motor_data->target_position -= 0.01;
        // }
        // test_3508[0].motor_data->tor = 0.5;
        // test_3508[0].motor_data->kp = 0;
        // test_3508[0].motor_data->kd = 0;
        //motor_control_pos_vel_acc(&hfdcan2, 1, 5000, 1000, 40);
        // 根据遥控器输入更新底盘控制指令
        RemoteControltest(&test_cmd_send);
        // 更新底盘电机状态
        test_motor_updata(test_3508, test_4310, test_EL05);
        usart_vofa_send(&huart7);
        // 控制任务执行频率（2ms周期）
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 底盘初始化函数
 * @param motors 底盘电机实例数组指针
 * @param test_Cmd 底盘控制命令结构体指针
 * @details 初始化底盘各电机的PID控制器参数
 */
void test_Init(MotorInstance *motors, test_Ctrl_Cmd_s *test_Cmd)
{   
    //底盘3508单环PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 5.0f, 0.0, 0.0, 0.0f, 50.0f, 9000.0f, 0.001f, 0.2f, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 15.0f, 0.0f, 2.0f, 0.4f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_Init(&motors[1].motor_data->pid[single_loop], 23.0f, 0.0, 0.0, 5.0f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 15.0f, 0.0f, 2.0f, 0.4f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors[2].motor_data->pid[single_loop], 23.0f, 0.0, 0.0, 5.0f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors[2].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors[2].motor_data->pid[cascade_outer], 15.0f, 0.0f, 2.0f, 0.4f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    PID_Init(&motors[3].motor_data->pid[single_loop], 23.0f, 0.0, 0.0, 5.0f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors[3].motor_data->pid[cascade_inner], 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors[3].motor_data->pid[cascade_outer], 15.0f, 0.0f, 2.0f, 0.4f, 50.0f, 10000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    //底盘跟随云台旋转PID控制器初始化
    PID_Init(&test_Cmd->cascade_pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);//未使用
    PID_Init(&test_Cmd->cascade_pid[cascade_inner], 28.0f, 0.0f, 0.0f, 0.0f, 10.0f, 1000.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&test_Cmd->cascade_pid[cascade_outer], 1.0f, 0.0f, 2.0f, 0.0f, 50.0f, 100.0f, 0.001f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

        // MATLAB离线计算的K矩阵 (默认)
  static const float K[2][4] = {
      {0.5029617493, 0.0215646166,    0.0f,     0.0f    },  // K[0][1] 置零
      {0.0f,      0.0f,   10.0f,    3.2032f  }
  };
    LqrBasic_Init(&lqr, K, 1.0f, 1.0f);
    //PID清零
    test_PIDClear(test_3508);
}

/**
 * @brief 设置底盘电机状态
 * @param status 电机状态（启用或禁用）
 * @param motors 底盘电机实例数组指针
 * @param motor_count 电机数量
 */
void test_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count)
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
 * @param test_Cmd 底盘控制命令结构体指针
 * @details 根据遥控器开关状态更新底盘工作模式
 */
void test_mode_update(test_Ctrl_Cmd_s *test_Cmd)
{
    if(test_Cmd == NULL)
    {
        return;
    }
    // 如果SBUS信号断开，则强制进入零力模式
    if(sbus_online == 0)
    {
        test_Cmd->test_mode = test_ZERO_FORCE;
        return;
    }
    // 根据遥控器左侧开关位置选择底盘模式
    switch (left_switch)
    {
    case switch_down:
        test_Cmd->test_mode = test_NO_FOLLOW;
        break; 
    case switch_mid:
        test_Cmd->test_mode = test_FOLLOW_GIMBAL_YAW;
        break;
    case switch_up:
        test_Cmd->test_mode = test_ROTATE;
        break;    
    default:
        test_Cmd->test_mode = test_ZERO_FORCE;
        break;
    }
}

/**
 * @brief 更新底盘电机控制
 * @param motors 底盘电机实例数组指针
 * @details 执行电机控制算法并更新电机输出
 */
void test_motor_updata(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3)
{
    if(motors == NULL)
    {
        return;
    }
    //更新底盘3508电机控制输出
    motors->motor_control(motors);
    motors2->motor_control(motors2);
    motors3->motor_control(motors3);
}

/**
 * @brief 根据遥控器输入控制底盘运动
 * @param test_Cmd 底盘控制命令结构体指针
 * @details 解析遥控器通道数据，计算底盘在不同模式下的运动指令
 */
void RemoteControltest(test_Ctrl_Cmd_s *test_Cmd) 
{
    if(test_Cmd == NULL)
    {
        return;
    }
    /***********************************确定底盘四个电机的目标速度*****************************************/
    // 设置底盘跟随云台旋转的速度
    switch(test_Cmd->test_mode) 
    {
        case test_FOLLOW_GIMBAL_YAW:     //跟随云台模式
            test_Cmd->vx = -(float)rc_ctrl.rc.ch[1]; //前后移动量（遥控器通道1取反）
            test_Cmd->vy = -(float)rc_ctrl.rc.ch[0]; //左右移动量（遥控器通道0取反）
            test_Cmd->w = 0;                         // 角速度由跟随算法提供
            break;
        case test_NO_FOLLOW:             //不跟随云台模式
            test_Cmd->vx = -(float)rc_ctrl.rc.ch[1]; //前后移动量
            test_Cmd->vy = -(float)rc_ctrl.rc.ch[0]; //左右移动量
            test_Cmd->w = 0;                         // 无角速度
            break;
        case test_ROTATE:		        //小陀螺模式（自旋）
            test_Cmd->vx = -(float)rc_ctrl.rc.ch[1]; //前后移动量
            test_Cmd->vy = -(float)rc_ctrl.rc.ch[0]; //左右移动量
            test_Cmd->w = 2;
            break;
	    case test_ZERO_FORCE:		    //零电流模式
            test_Cmd->vx = 0;            // 无前后移动
            test_Cmd->vy = 0;            // 无左右移动
            test_Cmd->w = 0;             // 无旋转
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
void test_PIDClear(MotorInstance  *motors)
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
    PID_Clear(test_cmd_send.cascade_pid);
}

#endif
