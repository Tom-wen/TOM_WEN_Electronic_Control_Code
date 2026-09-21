/**
 * @file arm_lift.c
 * @brief 机械臂升降机构控制模块
 * @details 控制机械臂的升降（3508电机）和夹爪（2006电机）运动
 */

#include "arm_lift.h"

#ifdef COMPILE_ARM_LIFT

/* 升降机构控制命令全局变量 */
Arm_Lift_Ctrl_Cmd_s arm_lift_cmd_send;

/**
 * @brief 升降机构任务入口函数
 * @param argument FreeRTOS任务参数（未使用）
 * @note 任务周期2ms，负责升降机构的初始化和控制循环
 */
void arm_lift_task(void *argument)
{
    /* 初始化升降机构电机（3508用于升降，2006用于夹爪） */
    Arm_Lift_Init(Arm_3508, Arm_2006);
    
    for(;;)
    {
        /* 更新升降机构工作模式 */
        Arm_Lift_mode_update(&arm_lift_cmd_send);
        
        /* 根据模式执行相应控制 */
        switch (arm_lift_cmd_send.lift_mode)
        {
            case ARM_LIFT_ZERO_FORCE:
                /* 零力模式：禁用电机，允许手动调整 */
                Arm_Lift_Motor_Status(Motor_Disable, Arm_3508, Arm_2006, 1);
                break;
                
            case ARM_LIFT_NORMAL:
                /* 正常模式：使能电机并执行控制 */
                Arm_Lift_Motor_Status(Motor_Enable, Arm_3508, Arm_2006, 1);
                Arm_Lift_control(&arm_lift_cmd_send);           // 计算控制量
                Arm_Lift_Motor_Set(Arm_3508, Arm_2006, &arm_lift_cmd_send);  // 设置电机目标值
                break;  
            case ARM_LIFT_REMOTE:
                /* 正常模式：使能电机并执行控制 */
                Arm_Lift_Motor_Status(Motor_Enable, Arm_3508, Arm_2006, 1);
                Arm_Lift_control(&arm_lift_cmd_send);           // 计算控制量
                Arm_Lift_Motor_Set(Arm_3508, Arm_2006, &arm_lift_cmd_send);  // 设置电机目标值
                break;               
        }
       
        /* 发送电机控制指令 */
        arm_lift_motor_updata(Arm_3508, Arm_2006);
        
        /* 任务延时2ms */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 升降机构电机PID参数初始化
 * @param motors1 3508电机实例指针（用于升降控制）
 * @param motors2 2006电机实例指针（用于夹爪控制）
 */
void Arm_Lift_Init(MotorInstance *motors1, MotorInstance *motors2)
{   
    /* 升降3508电机PID初始化 */
    PID_Init(&motors1[0].motor_data->pid[single_loop], 30.0f, 0.0f, 0.0f, 0.0f, 10.0f, 12000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    PID_Init(&motors1[0].motor_data->pid[cascade_inner], 10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors1[0].motor_data->pid[cascade_outer], 1.0f, 0.0f, 4.0f, 0.0f, 50.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    /* 夹爪2006电机PID初始化 */
    // 单环PID：未使用，参数全为0
    PID_Init(&motors2[0].motor_data->pid[single_loop], 10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 6000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 1.4f, 0.0f, 0.0f, 0.0f, 50.0f, 70.0f, 0.002f, 0.2f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
}

/**
 * @brief 设置升降机构电机使能状态
 * @param status 电机状态（Motor_Enable启用 / Motor_Disable禁用）
 * @param motors 3508电机实例指针（升降电机）
 * @param motors1 2006电机实例指针（夹爪电机）
 * @param motor_count 电机数量
 */
void Arm_Lift_Motor_Status(Motor_status status, MotorInstance *motors, MotorInstance *motors1, uint8_t motor_count)
{
    switch (status)
    {
    case Motor_Enable:
        /* 使能所有电机 */
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_enable(motors[i].motor_data);
            DJI_Motor_enable(motors1[i].motor_data);
        }
        break;
    case Motor_Disable:
        /* 禁用所有电机 */
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
            DJI_Motor_disable(motors1[i].motor_data);
        }
        break;
    default:
        /* 默认禁用所有电机（安全保护） */
        for(int i = 0; i < motor_count; i++)
        {
            DJI_Motor_disable(motors[i].motor_data);
            DJI_Motor_disable(motors1[i].motor_data);
        }
        break;
    }
}

/**
 * @brief 更新升降机构工作模式
 * @param lift_Cmd 升降机构控制命令结构体指针
 * @note 根据遥控器在线状态自动切换模式：
 *       - 遥控器离线时进入零力模式（安全保护）
 *       - 遥控器在线时进入正常工作模式
 */
void Arm_Lift_mode_update(Arm_Lift_Ctrl_Cmd_s *lift_Cmd)
{
    /* 空指针检查 */
    if(lift_Cmd == NULL)
    {
        return;
    }
    
    /* 根据编译目标和遥控器状态设置工作模式 */
    #ifdef chassis_board
        /* 底盘板编译配置 */
        if(received_data.sbus_online == 0)
        {
            lift_Cmd->lift_mode = ARM_LIFT_ZERO_FORCE;  /* 遥控器离线，进入零力模式 */
            return;
        }
        switch (received_left_switch)
        {
            case switch_down:
                lift_Cmd->lift_mode = ARM_LIFT_NORMAL;
                break;
            case switch_mid:
                lift_Cmd->lift_mode = ARM_LIFT_REMOTE;
                break;
            default:
                break;
        }
    #else
        /* 其他板编译配置 */
        if(received_data.sbus_online == 0)
        {
            lift_Cmd->lift_mode = ARM_LIFT_ZERO_FORCE;  /* 遥控器离线，进入零力模式 */
            return;
        }
        else{
            lift_Cmd->lift_mode = ARM_LIFT_NORMAL;      /* 遥控器在线，正常工作 */
        }
    #endif
}

/**
 * @brief 发送升降机构电机控制指令
 * @param motors1 3508电机实例指针（升降电机）
 * @param motors2 2006电机实例指针（夹爪电机）
 * @note 调用电机控制回调函数发送电流指令到CAN总线
 */
void arm_lift_motor_updata(MotorInstance *motors1, MotorInstance *motors2)
{
    /* 空指针检查 */
    if(motors1 == NULL || motors2 == NULL)
    {
        return;
    }
    
    /* 升降3508电机发送电流指令 */
    motors1->motor_control(motors1);
    
    /* 夹爪2006电机发送电流指令 */
    motors2->motor_control(motors2);
}

/**
 * @brief 升降机构控制量计算函数
 * @param lift_Cmd 升降机构控制命令结构体指针
 * @note 按键映射：
 *       - Q键：升降抬升（速度+5000）
 *       - E键：升降下降（速度-5000）
 *       - F键：夹爪夹紧（速度-1000）
 *       - G键：夹爪松开（速度+1000）
 */
void Arm_Lift_control(Arm_Lift_Ctrl_Cmd_s *lift_Cmd)
{
    /* 升降电机速度控制：W键抬升，S键下降 */
    if(arm_lift_cmd_send.lift_mode == ARM_LIFT_REMOTE){
        lift_Cmd->lift_speed = (float)received_data.rc_ctrl.rc.ch[3] * 3;  
    }
    else{
        if(RECEIVE_KEY_W){
            lift_Cmd->lift_speed = 1100;    /* 抬升速度 */
        } 
        else if(RECEIVE_KEY_S){
            lift_Cmd->lift_speed = -800;   /* 下降速度 */
        } 
        else{
            lift_Cmd->lift_speed = 100;       /* 停止 */
        }
    }
    
    /* 夹爪电机速度控制：鼠标左键夹住，右键松开 */
    if(received_data.rc_ctrl.mouse_left){
        lift_Cmd->claw_speed = -800;   /* 夹住 */
    }
    else if(received_data.rc_ctrl.mouse_right){
        lift_Cmd->claw_speed = 800;    /* 松开 */
    }
    else{
        lift_Cmd->claw_speed = 0;
    }

}

/**
 * @brief 设置升降机构电机目标速度值
 * @param motors1 3508电机实例指针（升降电机）
 * @param motors2 2006电机实例指针（夹爪电机）
 * @param lift_Cmd 升降机构控制命令结构体指针
 * @note 将计算好的目标速度写入电机数据结构
 */
void Arm_Lift_Motor_Set(MotorInstance *motors1, MotorInstance *motors2, Arm_Lift_Ctrl_Cmd_s *lift_Cmd)
{
    /* 空指针检查 */
    if(motors1 == NULL || motors2 == NULL || lift_Cmd == NULL)
    {
        return;
    }
    
    /* 设置电机目标速度 */
    // 【BUG修复】原循环4次会越界访问，改为1次（仅控制单个电机）
    for(int i = 0; i < 1; i++)
    {
        motors1[i].motor_data->target_velocity = lift_Cmd->lift_speed;  /* 升降电机目标速度 */
        motors2[i].motor_data->target_velocity = lift_Cmd->claw_speed;  /* 夹爪电机目标速度 */
    }
}

#endif
