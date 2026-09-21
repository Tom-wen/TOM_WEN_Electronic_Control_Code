#include "arm.h"
#include "user_lib.h"

#ifdef COMPILE_ARM

Arm_Ctrl_Cmd_s arm_cmd_send;

void arm_task(void *argument)
{
    Arm_Init(Arm_4310, Arm_EL05, Arm_6036);
    DM_Motor_Init(Arm_4310);                       // DM系列电机初始化
    RS_Motor_Init(Arm_EL05, rs_vel_mode);
    GQ_Motor_Init(Arm_6036);
    for(;;)
    {
        switch (arm_cmd_send.arm_mode)
        {
            case ARM_ZERO_FORCE:
                Arm_Motor_Status(Motor_Disable, Arm_4310, Arm_EL05, Arm_6036);
                break;
            case ARM_CONTROL_MODE:
                Arm_Motor_Status(Motor_Enable, Arm_4310, Arm_EL05, Arm_6036);
                Arm_control(&arm_cmd_send);
                Arm_Motor_Set(&arm_cmd_send, Arm_4310, Arm_EL05, Arm_6036);
                break;  
            case ARM_REMOTE_MODE:
                Arm_Motor_Status(Motor_Enable, Arm_4310, Arm_EL05, Arm_6036);
                Arm_control(&arm_cmd_send);
                Arm_Motor_Set(&arm_cmd_send, Arm_4310, Arm_EL05, Arm_6036);
                break;                                          
        }
        Arm_mode_update(&arm_cmd_send); 
        arm_motor_updata(Arm_4310, Arm_EL05, Arm_6036);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

//云台初始化
void Arm_Init(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3)
{
    //4310PID初始化
    PID_Init(&motors[0].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors[0].motor_data->pid[cascade_inner], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors[0].motor_data->pid[cascade_outer], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);

    PID_Init(&motors[1].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors[1].motor_data->pid[cascade_inner], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors[1].motor_data->pid[cascade_outer], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
       
    //EL05PID初始化
    PID_Init(&motors2[0].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_inner], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[0].motor_data->pid[cascade_outer], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);

    PID_Init(&motors2[1].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[1].motor_data->pid[cascade_inner], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    PID_Init(&motors2[1].motor_data->pid[cascade_outer], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_DISABLE);
    
    //6036PID初始化
    PID_Init(&motors3[0].motor_data->pid[single_loop], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors3[0].motor_data->pid[cascade_inner], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    PID_Init(&motors3[0].motor_data->pid[cascade_outer], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PID_D_First_ENABLE);
    //初始角度赋值
    arm_cmd_send.Arm6036_Motor[0].arm_position = 0;
    arm_cmd_send.Arm6036_Motor[0].arm_current = 200; 
    arm_cmd_send.Arm6036_Motor[0].arm_speed = 1500;    
    arm_cmd_send.Arm4310_Motor[0].arm_position = ARM4310_J2_START_POS;//旋转关节
    arm_cmd_send.Arm4310_Motor[0].arm_current = 1.0; 
    arm_cmd_send.Arm4310_Motor[0].arm_speed = 0.4;
    arm_cmd_send.Arm4310_Motor[1].arm_position = -1.24;//大关节
    arm_cmd_send.Arm4310_Motor[1].arm_current = 1.0; 
    arm_cmd_send.Arm4310_Motor[1].arm_speed = 0.4;
    arm_cmd_send.Claw_Motor[0].claw_position = 0;
    arm_cmd_send.Claw_Motor[0].claw_current = 5.0; 
    arm_cmd_send.Claw_Motor[0].claw_speed = 0;
    arm_cmd_send.Claw_Motor[1].claw_position = 0;
    arm_cmd_send.Claw_Motor[1].claw_current = 5.0; 
    arm_cmd_send.Claw_Motor[1].claw_speed = 0;    
}

//底盘电机状态设置
void Arm_Motor_Status(Motor_status status, MotorInstance *motors, MotorInstance *motors1, MotorInstance *motors2)
{
    switch (status)
    {
    case Motor_Enable:
        for(int i = 0; i < 2; i++)
        {
            DM_Motor_enable(motors[i].motor_data);
            RS_Motor_enable(motors1[i].motor_data);
        }
        GQ_Motor_enable(motors2[0].motor_data);
        break;
    case Motor_Disable:
        for(int i = 0; i < 2; i++)
        {
            DM_Motor_disable(motors[i].motor_data);
            RS_Motor_disable(motors1[i].motor_data);
        }
        GQ_Motor_disable(motors2[0].motor_data);
        break;
    default:
        for(int i = 0; i < 2; i++)
        {
            DM_Motor_disable(motors[i].motor_data);
            RS_Motor_disable(motors1[i].motor_data);
        }
        GQ_Motor_disable(motors2[0].motor_data);
        break;
    }
}

//机械臂电机状态更新
void arm_motor_updata(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3)
{
    if(motors == NULL)
    {
        return;
    }
    //机械臂力矩控制
    motors->motor_control(motors);
    motors2->motor_control(motors2);
    motors3->motor_control(motors3);
}

void RemoteControlArm(Arm_Ctrl_Cmd_s *Arm_Cmd) 
{
    if(Arm_Cmd == NULL)
    {
        return;
    }
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch(Arm_Cmd->arm_mode) 
    {
        case ARM_ZERO_FORCE:     //零电流模式
            for(int i = 0; i < 2; i++)
            {
                Arm_Cmd->Arm4310_Motor[i].arm_current = 0.0f;
                Arm_Cmd->Arm4310_Motor[i].arm_position = 0.0f;
                Arm_Cmd->Arm4310_Motor[i].arm_speed = 0.0f;
                Arm_Cmd->Claw_Motor[i].claw_current = 0.0f;
                Arm_Cmd->Claw_Motor[i].claw_position = 0.0f;
                Arm_Cmd->Claw_Motor[i].claw_speed = 0.0f;
            }
            Arm_Cmd->Arm6036_Motor[0].arm_current = 0.0f;
            Arm_Cmd->Arm6036_Motor[0].arm_position = 0.0f;
            Arm_Cmd->Arm6036_Motor[0].arm_speed = 0.0f;
            break;          
        default:
            break;
    }
}

//遥控器控制机械臂模式更新
void Arm_mode_update(Arm_Ctrl_Cmd_s *Arm_Cmd)
{
    if(Arm_Cmd == NULL)
    {
        return;
    }
    if(sbus_online == 0)
    {
        Arm_Cmd->arm_mode = ARM_ZERO_FORCE;
        return;
    }
    #ifdef DJI_REMOTE
        switch (left_switch)
        {
            case switch_down:
                Arm_Cmd->arm_mode = ARM_ZERO_FORCE;
                break; 
                break;
            case switch_up:
                Arm_Cmd->arm_mode = ARM_CONTROL_MODE;
                break;    
            default:
                break;
        }
    #endif
    #ifdef FS_REMOTE
        switch (left_switch)
        {
            case switch_down:
                Arm_Cmd->arm_mode = ARM_CONTROL_MODE;
                break; 
                break;    
            default:
                Arm_Cmd->arm_mode = ARM_ZERO_FORCE;
                break;
        }
    #endif 
    #ifdef VT_03_REMOTE
        switch (left_switch)
        {
            case switch_down:
                Arm_Cmd->arm_mode = ARM_CONTROL_MODE;
                break;
            case switch_mid:
                Arm_Cmd->arm_mode = ARM_REMOTE_MODE;
                break;
            default:
                break;
        }

    #endif 
}

void Arm_control(Arm_Ctrl_Cmd_s *Arm_Cmd)
{
    //高擎控制
    if(Arm_Cmd->Arm6036_Motor[0].arm_position >= ARM6036_MAX_POS){
        Arm_Cmd->Arm6036_Motor[0].arm_position = ARM6036_MAX_POS;
    }
    else if(Arm_Cmd->Arm6036_Motor[0].arm_position <= ARM6036_MIN_POS){
        Arm_Cmd->Arm6036_Motor[0].arm_position = ARM6036_MIN_POS;
    }
    
    if(Arm_Cmd->arm_mode == ARM_REMOTE_MODE){
        Arm_Cmd->Arm6036_Motor[0].arm_current = 200; 
        Arm_Cmd->Arm6036_Motor[0].arm_speed = 1500;
        Arm_Cmd->Arm6036_Motor[0].arm_position = (float)rc_ctrl.rc.ch[2] * 3;   
    }
    else{
        if(KEY_A){
            Arm_Cmd->Arm6036_Motor[0].arm_current = 200; 
            Arm_Cmd->Arm6036_Motor[0].arm_speed = 1500;
            Arm_Cmd->Arm6036_Motor[0].arm_position += 30;
        }
        else if(KEY_D){
            Arm_Cmd->Arm6036_Motor[0].arm_current = 200; 
            Arm_Cmd->Arm6036_Motor[0].arm_speed = 1500;
            Arm_Cmd->Arm6036_Motor[0].arm_position -= 30;
        }
    }

    //达妙电机J1控制
    if(Arm_Cmd->Arm4310_Motor[1].arm_position >= ARM4310_J1_MAX_POS){
        Arm_Cmd->Arm4310_Motor[1].arm_position = ARM4310_J1_MAX_POS;
    }
    else if(Arm_Cmd->Arm4310_Motor[1].arm_position <= ARM4310_J1_MIN_POS){
        Arm_Cmd->Arm4310_Motor[1].arm_position = ARM4310_J1_MIN_POS;
    }
    if(KEY_Q){
        Arm_Cmd->Arm4310_Motor[1].arm_current = 1.0; 
        Arm_Cmd->Arm4310_Motor[1].arm_speed = 0.4;
        Arm_Cmd->Arm4310_Motor[1].arm_position += 0.01;
    }
    else if(KEY_E){
        Arm_Cmd->Arm4310_Motor[1].arm_current = 1.0; 
        Arm_Cmd->Arm4310_Motor[1].arm_speed = 0.4;
        Arm_Cmd->Arm4310_Motor[1].arm_position -= 0.01;
    }  
    //达妙电机J2控制
    if(Arm_Cmd->Arm4310_Motor[0].arm_position >= ARM4310_J2_MAX_POS){
        Arm_Cmd->Arm4310_Motor[0].arm_position = ARM4310_J2_MAX_POS;
    }
    else if(Arm_Cmd->Arm4310_Motor[0].arm_position <= ARM4310_J2_MIN_POS){
        Arm_Cmd->Arm4310_Motor[0].arm_position = ARM4310_J2_MIN_POS;
    }
    if(KEY_Z){
        Arm_Cmd->Arm4310_Motor[0].arm_current = 1.0; 
        Arm_Cmd->Arm4310_Motor[0].arm_speed = 0.4;
        Arm_Cmd->Arm4310_Motor[0].arm_position += 0.01;
    }
    else if(KEY_X){
        Arm_Cmd->Arm4310_Motor[0].arm_current = 1.0; 
        Arm_Cmd->Arm4310_Motor[0].arm_speed = 0.4;
        Arm_Cmd->Arm4310_Motor[0].arm_position -= 0.01;
    }   
    //灵足电机控制（互斥模式）
    if(KEY_R){  // 上：两个电机同向转
        Arm_Cmd->Claw_Motor[0].claw_speed = -3;
        Arm_Cmd->Claw_Motor[1].claw_speed = 3;
    }
    else if(KEY_F){  // 下：两个电机同向反向转
        Arm_Cmd->Claw_Motor[0].claw_speed = 3;
        Arm_Cmd->Claw_Motor[1].claw_speed = -3;
    }
    else if(KEY_G){  // 左：两个电机反向转（电机0正转，电机1反转）
        Arm_Cmd->Claw_Motor[0].claw_speed = -3;
        Arm_Cmd->Claw_Motor[1].claw_speed = -3;
    }
    else if(KEY_V){  // 右：两个电机反向转（电机0反转，电机1正转）
        Arm_Cmd->Claw_Motor[0].claw_speed = 3;
        Arm_Cmd->Claw_Motor[1].claw_speed = 3;
    }
    else{  // 停止
        Arm_Cmd->Claw_Motor[0].claw_speed = 0;
        Arm_Cmd->Claw_Motor[1].claw_speed = 0;
    }

    Arm_Cmd->Claw_Motor[0].claw_position = 0; //右边减少
    Arm_Cmd->Claw_Motor[1].claw_position = 0; //左边增加  
    Arm_Cmd->Claw_Motor[0].claw_current = 5; //右边电机,数组0
    Arm_Cmd->Claw_Motor[1].claw_current = 5; //左边电机,数组1
}

void Arm_Motor_Set(Arm_Ctrl_Cmd_s *Arm_Cmd, MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3)
{
    //达妙
    motors[0].motor_data->vel = Arm_Cmd->Arm4310_Motor[0].arm_speed;
    motors[0].motor_data->pos = Arm_Cmd->Arm4310_Motor[0].arm_position;
    motors[0].motor_data->cur = Arm_Cmd->Arm4310_Motor[0].arm_current;    
    motors[1].motor_data->vel = Arm_Cmd->Arm4310_Motor[1].arm_speed;
    motors[1].motor_data->pos = Arm_Cmd->Arm4310_Motor[1].arm_position;
    motors[1].motor_data->cur = Arm_Cmd->Arm4310_Motor[1].arm_current;
    //灵足电机
    motors2[0].motor_data->target_velocity = Arm_Cmd->Claw_Motor[0].claw_speed;
    motors2[0].motor_data->target_position = Arm_Cmd->Claw_Motor[0].claw_position;
    motors2[0].motor_data->target_current = Arm_Cmd->Claw_Motor[0].claw_current;
    motors2[1].motor_data->target_velocity = Arm_Cmd->Claw_Motor[1].claw_speed;
    motors2[1].motor_data->target_position = Arm_Cmd->Claw_Motor[1].claw_position;
    motors2[1].motor_data->target_current = Arm_Cmd->Claw_Motor[1].claw_current;
    //高擎
    motors3[0].motor_data->target_velocity = Arm_Cmd->Arm6036_Motor[0].arm_speed;
    motors3[0].motor_data->target_position = Arm_Cmd->Arm6036_Motor[0].arm_position;
    motors3[0].motor_data->target_current = Arm_Cmd->Arm6036_Motor[0].arm_current;
    
}

//自定义控制器控制函数
void User_define_control(float *angle, uint16_t *motor_target_angle)
{
    // angle[0] = -linear_map_clamped(motor_target_angle[ARM_Motor1], 20, 239, -1.73, 1.43);
    // angle[1] = linear_map_clamped(motor_target_angle[ARM_Motor2], 0, 261, -2.45, 1.56);
    // angle[2] = linear_map_clamped(motor_target_angle[ARM_Motor3], 0, 179, -0.61, 2.16);
}

#endif
