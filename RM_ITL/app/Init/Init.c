#include "Init.h"

/**
 * @brief 底盘电机结构体实例定义
 * @details 根据编译条件定义不同底盘类型的电机实例
 */

#ifdef COMPILE_MECANUM_CHASSIS
// 麦克纳姆轮底盘：仅定义3508电机4个
MotorInstance Chassis_3508[4] = {0};    // 底盘3508电机数组
MotorControlData chassis_control_data[4] = {0};     // 底盘3508电机控制数据
#endif

/**
 * @brief 云台电机结构体实例定义
 * @details 定义云台6020电机2个（YAW轴和PITCH轴）
 */
#ifdef COMPILE_GIMBAL
MotorInstance Gimbal_6020[2] = {0};     // 云台6020电机数组
MotorControlData gimbal_control_data[2] = {0};      // 云台电机控制数据
#endif


/**
 * @brief 发射机构电机结构体实例定义
 * @details 定义发射机构电机（shoot电机2个和trigger1个）
 */
#ifdef COMPILE_SHOOT
MotorInstance Shoot_3508[2] = {0};     // shoot3508电机数组
MotorInstance Trigger_2006[1] = {0};     // shoot3508电机数组
MotorControlData shoot_control_data1[2] = {0};      // shoot3508电机控制数据
MotorControlData shoot_control_data2[1] = {0};      // shoot3508电机控制数据
#endif


/**
 * @brief 电机控制数据存储数组
 * @details 为各类电机分配控制数据存储空间
 */

/**
 * @brief 机器人初始化函数
 * @details 执行机器人的完整初始化流程，包括BSP初始化、模块初始化和应用初始化
 */
void Robot_Init(void)
{
    // BSP层初始化（基础硬件抽象层）
    buzzer_init();                                
    DWT_Init(480);                                  // DWT（数据观察点和跟踪）初始化，480MHz
    bsp_can_init();                                 // CAN总线初始化
    usart_rx_dma_start(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));  // USART1 DMA接收初始化
    usart_rx_dma_start(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));  // USART2 DMA接收初始化
    usart_rx_dma_start(&huart5, sbus_buf, sizeof(sbus_buf));            // SBUS遥控器DMA接收初始化
    usart_rx_dma_start(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART_BUFFER_SIZE);  // USART3 DMA接收初始化
    usart_rx_dma_start(&huart7, usart7_rx_buf, sizeof(usart7_rx_buf));  // USART7 DMA接收初始化
    usart_rx_dma_start(&huart10, usart10_rx_buf, sizeof(usart10_rx_buf));  // USART10 DMA接收初始化
    // Modules层初始化（模块驱动层）
    remote_control_init();                         // 遥控器初始化

    // App层初始化（应用层）
    Motor_Init();                                  // 电机系统初始化
    
    Chassis_Power_Init();

    while(BMI088_init() != BMI088_NO_ERROR);       // BMI088 IMU传感器初始化（等待初始化成功）
}

/**
 * @brief 所有电机初始化函数
 * @details 根据编译条件调用相应的底盘、云台和机械臂电机初始化函数
 */
void Motor_Init()
{
    
    //麦轮底盘电机初始化
    #ifdef COMPILE_MECANUM_CHASSIS
    Chassis_Motor_Init(Chassis_3508, chassis_control_data);
    #endif
    
    //步兵云台初始化
    #ifdef COMPILE_GIMBAL
    Gimbal_Motor_Init(Gimbal_6020, gimbal_control_data);                        
    #endif


    //发射机构初始化
    #ifdef COMPILE_SHOOT
    Shoot_Motor_Init(Shoot_3508, Trigger_2006, shoot_control_data1, shoot_control_data2);                        
    #endif


}


/**
 * @brief 麦克纳姆轮底盘电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化麦克纳姆轮底盘的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_MECANUM_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
    // 3508电机初始化 - 使用CAN2总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
}
#endif

/**
 * @brief 云台电机初始化函数
 * @param[in,out] motors 6020电机实例数组指针
 * @param[in,out] motors_data 6020电机控制数据数组指针
 * @details 初始化云台的6020电机（YAW轴和PITCH轴），配置CAN通信和控制模式
 */
#ifdef COMPILE_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
    // 6020电机初始化 - 使用CAN3总线
    // YAW轴（偏航轴）电机，ID=7
    motors[0] = CreateMotor(Motor6020V, 7, &motors_data[0], 2, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    // PITCH轴（俯仰轴）电机，ID=6
    motors[1] = CreateMotor(Motor6020V, 6, &motors_data[1], 2, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN1);
}
#endif





#ifdef COMPILE_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN3总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 2, &motors_data1[0], 2, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
    //右摩擦轮
    motors[1] = CreateMotor(Motor3508, 3, &motors_data1[1], 2, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
    // 2006电机初始化 - 使用CAN1总线
    motors2[0] = CreateMotor(Motor2006, 1, &motors_data2[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN1);
}
#endif




/**
 * @brief 创建电机实例函数
 * @param[in] type 电机类型（Motor3508、Motor6020等）
 * @param[in] id 电机ID
 * @param[in,out] motor_data 电机控制数据指针
 * @param[in] motor_count 同一发送函数下电机的数量（用于CAN报文打包）
 * @param[in] control_func 电机控制函数指针
 * @param[in] callback_func 电机CAN回调函数指针
 * @param[in] can_port CAN端口号
 * @return MotorInstance 创建的电机实例
 * @details 根据指定参数创建并初始化电机实例，配置反馈数据、CAN通信等
 */
MotorInstance CreateMotor(uint16_t type, uint8_t id, MotorControlData *motor_data, uint8_t motor_count,
                       void (*control_func)(MotorInstance *motors),                
                       void (*callback_func)(CANRxData *rx_data, CAN_PORT can_port), CAN_PORT can_port)
{
    // 初始化电机基本数据
    motor_data->id = id;                            // 设置电机ID
    motor_data->motor_enable = 0;                   // 使能电机
    motor_data->pos = 0.0;                          // 位置初始化为0
    motor_data->speed = 0.0;                          //线速度初始化为0
    motor_data->tor = 0.0;                          // 扭矩初始化为0
    motor_data->target_velocity = 0;                // 目标速度初始化为0
    motor_data->target_position = 0;                // 目标位置初始化为0
    motor_data->target_current = 0;                 // 目标电流初始化为0
    float first_deg = 0;
    // 根据电机类型配置反馈数据和角度计算
    switch (type)
    {
        case Motor3508:
            // 3508电机：使用DJI电机反馈数据
            motor_data->feedback = &DJI_Motor_RX[can_port][id-1];   // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0
            break;
            
        case Motor6020C:
            // 6020电机：使用DJI电机反馈数据
            motor_data->feedback = &DJI_Motor_RX[can_port][id+3];   // 设置反馈数据指针（ID偏移）
            first_deg = motor_data->feedback->pos / 22.75278f; // 将编码器值转换为角度
            motor_data->last_angle  = first_deg;                    // 设置上次角度
            motor_data->total_angle = first_deg;                    // 设置总角度
            break;  

        case Motor6020V:
            // 6020电机：使用DJI电机反馈数据
            motor_data->feedback = &DJI_Motor_RX[can_port][id+3];   // 设置反馈数据指针（ID偏移）
            first_deg = motor_data->feedback->pos / 22.75278f; // 将编码器值转换为角度
            motor_data->last_angle  = first_deg;                    // 设置上次角度
            motor_data->total_angle = first_deg;                    // 设置总角度
            break;    

        case Motor2006:
            // 2006电机：使用DJI电机反馈数据
            motor_data->feedback = &DJI_Motor_RX[can_port][id-1];   // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0
            break;
            
        case Motor4310:
            // 4310电机：使用DM电机反馈数据
            motor_data->feedback = &DM_Motor_RX[can_port][id-1];    // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0
            break; 

        case MotorM15:
            // M15电机：使用BM电机反馈数据
            motor_data->feedback = &DJI_Motor_RX[can_port][id-1];    // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0
            break; 

        case MotorRS:
            // MRS电机：使用BM电机反馈数据
            motor_data->feedback = &RS_Motor_RX[can_port][id-1];    // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0                
            break;

        case MotorGQ:
            // GQ电机：使用GQ电机反馈数据
            motor_data->feedback = &GQ_Motor_RX[can_port][id-1];    // 设置反馈数据指针
            motor_data->last_angle  = 0.0;                          // 上次角度初始化为0
            motor_data->total_angle = 0.0;                          // 总角度初始化为0  
            break;

        default:
            break;
    }

    // 根据CAN端口注册回调函数和设置CAN句柄
    switch(can_port) 
    {
        case CAN1:
            bsp_can1_set_callback(callback_func);       // 设置CAN1回调函数
            motor_data->hfdcan = &hfdcan1;              // 设置CAN1句柄
            break;
            
        case CAN2:
            bsp_can2_set_callback(callback_func);       // 设置CAN2回调函数
            motor_data->hfdcan = &hfdcan2;              // 设置CAN2句柄
            break;
            
        case CAN3:
            bsp_can3_set_callback(callback_func);       // 设置CAN3回调函数
            motor_data->hfdcan = &hfdcan3;              // 设置CAN3句柄
            break;
            
        default:
            bsp_can2_set_callback(callback_func);       // 默认使用CAN2回调函数
            break;
    }

    // 创建并初始化MotorInstance结构体
    MotorInstance motor = {0};          // 初始化电机实例结构体
    motor.motor_count = motor_count;    // 同一发送函数下电机的数量
    motor.type = type;                  // 设置电机类型
    motor.motor_data = motor_data;      // 设置电机控制数据指针
    motor.motor_control = control_func; // 设置电机控制函数指针
    return motor;                       // 返回创建的电机实例
}
/*
 * 弱函数默认实现 - 提供任务的默认空实现
 * Note:
 * - 为了确保代码同时适用于不同硬件配置（如单板/双板控制、不同底盘类型等），
 *   这里为所有任务提供弱函数默认实现。
 * - 若某个任务在其他文件中有强定义实现，则会自动覆盖这里的弱定义。
 * - 若某个任务未实现，则会执行弱定义版本，任务启动后立即删除自己，释放资源。
 * Warning:
 * - 不要在弱函数中添加任何实际功能代码，它们仅用于资源管理。
 * - 强定义的任务实现应该在对应的模块文件中（如 chassis.c, gimbal.c 等）。
 */
/**
 * @brief 机械臂任务弱定义
 * @note 如果没有实现机械臂功能，任务会自动删除
 */
__weak void arm_task(void *argument)
{
    vTaskDelete(NULL);  // 删除当前任务
}
/**
 * @brief INS姿态解算任务弱定义
 * @note 如果没有实现INS功能，任务会自动删除
 */
__weak void ins_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 底盘控制任务弱定义
 * @note 如果没有实现底盘功能，任务会自动删除
 */
__weak void chassis_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 设备离线检测任务弱定义
 * @note 如果没有实现离线检测功能，任务会自动删除
 */
__weak void detect_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 云台控制任务弱定义
 * @note 如果没有实现云台功能，任务会自动删除
 */
__weak void gimbal_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 数据处理任务弱定义
 * @note 如果没有实现数据处理功能，任务会自动删除
 */
__weak void data_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 测试任务弱定义
 * @note 如果没有使用测试功能，任务会自动删除
 */
__weak void test_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 发射机构任务弱定义
 * @note 如果没有使用发射机构功能，任务会自动删除
 */
__weak void shoot_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 电机控制任务弱定义
 * @note 如果没有使用电机控制功能，任务会自动删除
 */
__weak void motor_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 裁判系统任务弱定义
 * @note 如果没有使用裁判系统功能，任务会自动删除
 */
__weak void referee_task(void *argument)
{
    vTaskDelete(NULL);
}