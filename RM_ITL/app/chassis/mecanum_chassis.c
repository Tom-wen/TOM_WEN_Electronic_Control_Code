#include "mecanum_chassis.h"
#include "control_power.h"
#include "user_lib.h"

//底盘电机发送数据结构体实例
Chassis_Ctrl_Cmd_s chassis_cmd_move;

//函数声明
static void chassis_set_mode(void);
static void chassis_feedback_update(void);
static void chassis_set_contorl(void);
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, 
                                                   const float wz_set, float wheel_speed[4]);
 
/**
 * @brief 底盘主任务函数
 * @param argument 任务参数
 * @details 这是底盘控制的主任务函数，负责根据不同的模式控制底盘运动
 */
void chassis_task(void *argument)
{
    Chassis_Init(Chassis_3508, &chassis_cmd_move);
    for(;;)
    {
        /* 设置底盘控制模式 */
        chassis_set_mode();
        /* 底盘数据更新 */
        chassis_feedback_update();
        /* 底盘控制量设置 */
        chassis_set_contorl();
        /* 底盘控制 */
        chassis_control_loop();
        // 更新底盘电机状态
        chassis_motor_updata(Chassis_3508);
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
    PID_Init(&motors[0].motor_data->pid[single_loop], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_IOUT, CHASSIS_MOTOR_SPEED_MAX_OUT, 0.0f);

    PID_Init(&motors[1].motor_data->pid[single_loop], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_IOUT, CHASSIS_MOTOR_SPEED_MAX_OUT, 0.0f);

    PID_Init(&motors[2].motor_data->pid[single_loop], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_IOUT, CHASSIS_MOTOR_SPEED_MAX_OUT, 0.0f);

    PID_Init(&motors[3].motor_data->pid[single_loop], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_IOUT, CHASSIS_MOTOR_SPEED_MAX_OUT, 0.0f);
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
 * @brief 设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @retval none
 */
static void chassis_set_mode(void)
{
    chassis_behaviour_mode_set(&chassis_cmd_move);
}

/**
 * @brief 底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @retval none
 */
static void chassis_feedback_update(void)
{   
    for (uint8_t i = 0; i < 4; i++)
    {
        /* 更新电机速度，加速度是速度的PID微分 */
        Chassis_3508[i].motor_data->speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * 
        Chassis_3508[i].motor_data->feedback->vel;
    }
    /* 更新底盘纵向速度x，平移速度y，旋转速度wz，坐标系为右手系 */
    chassis_cmd_move.vx = (Chassis_3508[0].motor_data->speed - Chassis_3508[1].motor_data->speed - 
                      Chassis_3508[2].motor_data->speed + Chassis_3508[3].motor_data->speed) * 
                      MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_cmd_move.vy = (Chassis_3508[0].motor_data->speed + Chassis_3508[1].motor_data->speed - 
                      Chassis_3508[2].motor_data->speed - Chassis_3508[3].motor_data->speed) * 
                      MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_cmd_move.w = (-Chassis_3508[0].motor_data->speed - Chassis_3508[1].motor_data->speed - 
                       Chassis_3508[2].motor_data->speed - Chassis_3508[3].motor_data->speed) * 
                       MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}
/**
 * @brief 设置底盘控制设置值，三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @retval none
 */
static void chassis_set_contorl(void)
{
    chassis_behaviour_control_set(&chassis_cmd_move);
}




/**
 * @brief 控制循环，根据控制设定值，计算电机电流值，进行控制
 * @retval none
 */
static void chassis_control_loop(void)
{
    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* 麦轮运动分解 */
    chassis_vector_to_mecanum_wheel_speed(chassis_cmd_move.vx_set, chassis_cmd_move.vy_set, 
                                          chassis_cmd_move.w_set, wheel_speed);
    for (uint8_t i = 0; i < 4; i++)
    {
        // 根据计算的轮速设置电机目标速度
        Chassis_3508[i].motor_data->target_velocity = wheel_speed[i] / CHASSIS_MOTOR_RPM_TO_VECTOR_SEN; // 转换回电机转速单位
    }  
}
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, 
                                                   const float wz_set, float wheel_speed[4])
{
    wheel_speed[0] = vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

