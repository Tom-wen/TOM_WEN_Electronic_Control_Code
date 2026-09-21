#include "Init.h"

/**
 * @brief 底盘电机结构体实例定义
 * @details 根据编译条件定义不同底盘类型的电机实例
 */
#ifdef COMPILE_AGV_CHASSIS
// 舵轮底盘：定义3508和6020电机各4个
MotorInstance Chassis_3508[4] = {0};    // 底盘3508电机数组
MotorInstance Chassis_6020[4] = {0};    // 底盘6020电机数组
MotorControlData chassis_control_data[4] = {0};     // 底盘3508电机控制数据
MotorControlData chassis_control_data2[4] = {0};    // 底盘6020电机控制数据
#endif

#ifdef COMPILE_HERO_CHASSIS
// 舵轮底盘：定义3508和6020电机各4个
MotorInstance Chassis_3508[4] = {0};    // 底盘3508电机数组
MotorInstance Chassis_6020[4] = {0};    // 底盘6020电机数组
MotorControlData chassis_control_data[4] = {0};     // 底盘3508电机控制数据
MotorControlData chassis_control_data2[4] = {0};    // 底盘6020电机控制数据
#endif

#ifdef COMPILE_M_HERO_CHASSIS
// 麦克纳姆轮底盘：仅定义3508电机4个
MotorInstance Chassis_3508[4] = {0};    // 底盘3508电机数组
MotorControlData chassis_control_data[4] = {0};     // 底盘3508电机控制数据
#endif

#ifdef COMPILE_MECANUM_CHASSIS
// 麦克纳姆轮底盘：仅定义3508电机4个
MotorInstance Chassis_3508[4] = {0};    // 底盘3508电机数组
MotorControlData chassis_control_data[4] = {0};     // 底盘3508电机控制数据
#endif

//摆臂底盘注册电机
#ifdef ROBOTIC_SWING_CHASSIS
MotorInstance Chassis_3508[4] = {0};
MotorControlData chassis_control_data[4] = {0}; 

MotorInstance Hoisting_5047[4] = {0};
MotorControlData hoisting_control_data[4] = {0}; 
#endif

//靶车电机的注册
#ifdef TARGET_CHASSIS
MotorInstance Chassis_3508[2] = {0};
MotorControlData chassis_control_data[2] = {0}; 

MotorInstance Gimbal_5047[1] = {0};
MotorControlData gimbal_control_data[1] = {0};
#endif

//哨兵底盘注册电机
#ifdef AUTO_SENTRY
MotorInstance Chassis_3508[4] = {0};
MotorControlData chassis_control_data_1[4] = {0};
MotorInstance Chassis_6020[4] = {0};
MotorControlData chassis_control_data_2[4] = {0};
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
 * @brief 摆臂云台电机结构体实例定义
 * @details 定义云台6020电机2个（YAW轴和PITCH轴）
 */
#ifdef ROBOTIC_SWING_GIMBAL
MotorInstance Gimbal_Yaw[1] = {0};;
MotorInstance Gimbal_Pitch[1] = {0};;
MotorInstance Gimbal_6020[2] = {0};     // 云台6020电机数组
MotorControlData gimbal_control_data_Yaw[1] = {0};
MotorControlData gimbal_control_data_Pitch[1] = {0};      // 云台电机控制数据
MotorControlData gimbal_control_data[2] = {0};      // 云台电机控制数据
#endif

/**
 * @brief 哨兵云台电机结构体实例定义
 * @details 定义云台6020电机3个（YAW轴和PITCH轴）
 */
#ifdef AUTO_SENTRY
MotorInstance Gimbal_Big_Yaw_4310[1] = {0};                    // 云台4310电机数组
MotorInstance Gimbal_Small_Yaw_6020[1] = {0};                    // 云台4310电机数组
MotorInstance Gimbal_Pitch_4310[1] = {0};
MotorControlData gimbal_control_data[3] = {0};      // 云台电机控制数据
#endif

/**
 * @brief 无人机云台电机结构体实例定义
 * @details 定义云台6020电机2个（YAW轴和PITCH轴）
 */
#ifdef COMPILE_UAV_GIMBAL
MotorInstance Gimbal_6020[2] = {0};                 // 云台6020电机数组
MotorControlData gimbal_control_data[2] = {0};      // 云台电机控制数据
#endif

/**
 * @brief 英雄云台电机结构体实例定义
 * @details 定义云台6020电机2个（YAW轴和PITCH轴）
 */
#ifdef COMPILE_HERO_GIMBAL
MotorInstance Gimbal_4310[1] = {0};                    // 云台4310电机数组
MotorInstance Gimbal_3508[1] = {0};                    
MotorControlData gimbal_control_data[2] = {0};      // 云台电机控制数据
#endif

/**
 * @brief 机械臂电机结构体实例定义
 * @details 定义机械臂4310电机3个和2006电机4个
 */
#ifdef COMPILE_ARM
MotorInstance Arm_4310[2] = {0};                    // 机械臂4310电机数组
MotorInstance Arm_EL05[2] = {0};                    // 机械臂灵足电机数组
MotorInstance Arm_6036[1] = {0};                    // 机械臂高擎电机数组
MotorControlData arm_control_data[2] = {0};         // 机械臂4310电机控制数据
MotorControlData arm_control_data2[2] = {0};        // 机械臂灵足电机控制数据
MotorControlData arm_control_data3[1] = {0};        // 机械臂高擎电机控制数据
#endif

/**
 * @brief 机械臂抬升电机结构体实例定义
 * @details 定义机械臂4310电机3个和2006电机4个
 */
#ifdef COMPILE_ARM_LIFT
MotorInstance Arm_3508[1] = {0};                    // 机械臂抬升3508电机数组
MotorInstance Arm_2006[1] = {0};                    // 机械臂夹爪2006电机数组
MotorControlData arm_control_data[1] = {0};         // 机械臂抬升3508电机控制数据
MotorControlData arm_control_data2[1] = {0};        // 机械臂夹爪2006电机控制数据
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
 * @brief 摆臂发射机构电机结构体实例定义
 * @details 定义发射机构电机（shoot电机2个和trigger1个）
 */
#ifdef ROBOTIC_SWING_SHOOT
MotorInstance Shoot_3508[2] = {0};     // shoot3508电机数组
MotorInstance Trigger_2006[1] = {0};     // shoot3508电机数组
MotorControlData shoot_control_data1[2] = {0};      // shoot3508电机控制数据
MotorControlData shoot_control_data2[1] = {0};      // shoot3508电机控制数据
#endif


/**
 * @brief 无人机发射机构电机结构体实例定义
 * @details 定义发射机构电机（shoot电机2个和trigger1个）
 */
#ifdef COMPILE_UAV_SHOOT
MotorInstance Shoot_3508[2] = {0};     // shoot3508电机数组
MotorInstance Trigger_2006[1] = {0};     // shoot3508电机数组
MotorControlData shoot_control_data1[2] = {0};      // shoot3508电机控制数据
MotorControlData shoot_control_data2[1] = {0};      // shoot3508电机控制数据
#endif

/**
 * @brief 英雄发射机构电机结构体实例定义
 * @details 定义发射机构电机（shoot电机2个和trigger1个）
 */
#ifdef COMPILE_HERO_SHOOT
MotorInstance Shoot_3508[3] = {0};     // shoot3508电机数组
MotorInstance Trigger_3508[1] = {0};     // shoot3508电机数组
MotorControlData shoot_control_data1[3] = {0};      // shoot3508电机控制数据
MotorControlData shoot_control_data2[1] = {0};      // shoot3508电机控制数据
#endif

/**
 * @brief 自动哨兵发射机构电机结构体实例定义
 * @details 定义发射机构电机（shoot电机2个和trigger1个）
 */
#ifdef AUTO_SENTRY
MotorInstance Shoot_3508[2] = {0};     // shoot3508电机数组
MotorInstance Trigger_2006[1] = {0};     // shoot3508电机数组
MotorControlData shoot_control_data1[2] = {0};      // shoot3508电机控制数据
MotorControlData shoot_control_data2[1] = {0};      // shoot3508电机控制数据
#endif

#ifdef COMPILE_TEST
// 测试任务电机：仅定义3508电机4个
MotorInstance test_3508[2] = {0};    // 底盘3508电机数组
MotorInstance test_4310[3] = {0};    // 底盘3508电机数组
MotorInstance test_EL05[2] = {0};    // 底盘3508电机数组
MotorControlData test_control_data[7] = {0};     // 底盘3508电机控制数据
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
    //buzzer_init();                                // 蜂鸣器初始化（注释掉）
    DWT_Init(480);                                  // DWT（数据观察点和跟踪）初始化，480MHz
    bsp_can_init();                                 // CAN总线初始化
    WS2812_Init();                                  // 扩展板ws2812
    usart_rx_dma_start(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));  // USART1 DMA接收初始化
    usart_rx_dma_start(&huart5, sbus_buf, sizeof(sbus_buf));            // SBUS遥控器DMA接收初始化
    usart_rx_dma_start(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART_BUFFER_SIZE);  // USART3 DMA接收初始化
    usart_rx_dma_start(&huart7, usart7_rx_buf, sizeof(usart7_rx_buf));  // USART7 DMA接收初始化
    usart_rx_dma_start(&huart8, usart8_rx_buf, sizeof(usart8_rx_buf));  // USART9 DMA接收初始化
    usart_rx_dma_start(&huart9, usart9_rx_buf, sizeof(usart9_rx_buf));  // USART9 DMA接收初始化
    usart_rx_dma_start(&huart10, usart10_rx_buf, sizeof(usart10_rx_buf));  // USART10 DMA接收初始化
    MCP2515_Init(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin); //spi转can4初始化
    MCP2515_Init(&hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin); //spi转can5初始化
    MCP2515_Init(&hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin); //spi转can6初始化
    // Modules层初始化（模块驱动层）
    remote_control_init();                         // 遥控器初始化

    // App层初始化（应用层）
    Motor_Init();                                  // 电机系统初始化
  #ifdef COMPILE_POWER  
    Chassis_Power_Init();
  #endif
 
    while(BMI088_init() != BMI088_NO_ERROR);       // BMI088 IMU传感器初始化（等待初始化成功）
    
    //RTOS初始化
    RTOS_Init();
}

/**
 * @brief 所有电机初始化函数
 * @details 根据编译条件调用相应的底盘、云台和机械臂电机初始化函数
 */
void Motor_Init()
{
    //舵轮底盘电机初始化
    #ifdef COMPILE_AGV_CHASSIS
    Chassis_Motor_Init(Chassis_3508, Chassis_6020, chassis_control_data, chassis_control_data2);
    #endif
    
    //麦轮底盘电机初始化
    #ifdef COMPILE_MECANUM_CHASSIS
    Chassis_Motor_Init(Chassis_3508, chassis_control_data);
    #endif
    //麦轮底盘电机初始化
    #ifdef COMPILE_M_HERO_CHASSIS
    Chassis_Motor_Init(Chassis_3508, chassis_control_data);
    #endif

    //摆臂底盘电机初始化
    #ifdef ROBOTIC_SWING_CHASSIS
    Chassis_Motor_Init(Chassis_3508, chassis_control_data);
    #endif

    //英雄底盘电机初始化
    #ifdef COMPILE_HERO_CHASSIS
    Chassis_Motor_Init(Chassis_3508, Chassis_6020, chassis_control_data, chassis_control_data2);
    #endif
    
    //摆臂抬升电机初始化
    #ifdef ROBOTIC_SWING_HOISTING
    Hoisting_Motor_Init(Hoisting_5047, hoisting_control_data);
    #endif

    //哨兵底盘电机初始化
    #ifdef AUTO_SENTRY
    Chassis_Motor_Init(Chassis_3508 ,Chassis_6020, chassis_control_data_1,chassis_control_data_2);
    #endif

    //靶车初始化
    #ifdef TARGET_CHASSIS
    Target_Chassis_Motor_Init(Chassis_3508, chassis_control_data, Gimbal_5047, gimbal_control_data);
    #endif

    //机械臂初始化
    #ifdef COMPILE_ARM
    Arm_Motor_Init(Arm_4310, Arm_EL05, Arm_6036, arm_control_data, arm_control_data2, arm_control_data3);
    #endif

    //机械臂抬升初始化
    #ifdef COMPILE_ARM_LIFT
    Arm_LIFT_Motor_Init(Arm_3508, Arm_2006, arm_control_data, arm_control_data2);
    #endif

    //步兵云台初始化
    #ifdef COMPILE_GIMBAL
    Gimbal_Motor_Init(Gimbal_6020, gimbal_control_data);                        
    #endif

    //摆臂云台初始化
    #ifdef ROBOTIC_SWING_GIMBAL
    Gimbal_Yaw_Motor_Init(Gimbal_Yaw, gimbal_control_data_Yaw);
    Gimbal_Pitch_Motor_Init(Gimbal_Pitch, gimbal_control_data_Pitch);
    #endif

    //哨兵云台初始化
    #ifdef AUTO_SENTRY
    Gimbal_Motor_Init(Gimbal_Small_Yaw_6020, Gimbal_Big_Yaw_4310, Gimbal_Pitch_4310, gimbal_control_data);                        
    #endif

    //无人机云台初始化
    #ifdef COMPILE_UAV_GIMBAL
    Gimbal_Motor_Init(Gimbal_6020, gimbal_control_data);                        
    #endif

    //英雄云台初始化
    #ifdef COMPILE_HERO_GIMBAL
    Gimbal_Motor_Init(Gimbal_4310, Gimbal_3508, gimbal_control_data);                        
    #endif

    //发射机构初始化
    #ifdef COMPILE_SHOOT
    Shoot_Motor_Init(Shoot_3508, Trigger_2006, shoot_control_data1, shoot_control_data2);                        
    #endif

    //摆臂发射机构初始化
    #ifdef ROBOTIC_SWING_SHOOT
    Shoot_Motor_Init(Shoot_3508, Trigger_2006, shoot_control_data1, shoot_control_data2);                        
    #endif

    //无人机发射机构初始化
    #ifdef COMPILE_UAV_SHOOT
    Shoot_Motor_Init(Shoot_3508, Trigger_2006, shoot_control_data1, shoot_control_data2);                        
    #endif

    //英雄发射机构初始化
    #ifdef COMPILE_HERO_SHOOT
    Shoot_Motor_Init(Shoot_3508, Trigger_3508, shoot_control_data1, shoot_control_data2);                        
    #endif

    //哨兵发射机构初始化
    #ifdef AUTO_SENTRY
    Shoot_Motor_Init(Shoot_3508, Trigger_2006, shoot_control_data1, shoot_control_data2);                        
    #endif

    //测试任务初始化
    #ifdef COMPILE_TEST
    Test_Motor_Init(test_3508, test_4310, test_EL05, test_control_data);                        
    #endif

}

/**
 * @brief 舵轮底盘电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors1 6020电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @param[in,out] motors_data1 6020电机控制数据数组指针
 * @details 初始化舵轮底盘的3508和6020电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_AGV_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1)
{
    // 3508电机初始化 - 使用CAN2总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);

    // 6020电机初始化 - 使用CAN2总线
    motors1[0] = CreateMotor(Motor6020C, 1, &motors_data1[0], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[1] = CreateMotor(Motor6020C, 2, &motors_data1[1], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[2] = CreateMotor(Motor6020C, 3, &motors_data1[2], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[3] = CreateMotor(Motor6020C, 4, &motors_data1[3], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
}
#endif

#ifdef COMPILE_HERO_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1)
{
    // 3508电机初始化 - 使用CAN1总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
  
    // 6020电机初始化 - 使用CAN2总线
    motors1[0] = CreateMotor(Motor6020C, 1, &motors_data1[0], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[1] = CreateMotor(Motor6020C, 2, &motors_data1[1], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[2] = CreateMotor(Motor6020C, 3, &motors_data1[2], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[3] = CreateMotor(Motor6020C, 4, &motors_data1[3], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
}
#endif

#ifdef COMPILE_M_HERO_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
    // 3508电机初始化 - 使用CAN2总线
    #ifdef COMPILE_M_POWER
      motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2_MasterPower,DJI_motor_can_callback, CAN1);
      motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2_MasterPower,DJI_motor_can_callback, CAN1);
      motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2_MasterPower,DJI_motor_can_callback, CAN1);
      motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN1);
  #else
      motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
      motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
      motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
      motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN1);
  #endif
}
#endif

#ifdef AUTO_SENTRY
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1)
{
    // 3508电机初始化 - 使用CAN1总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);

    // 6020电机初始化 - 使用CAN2总线
    motors1[0] = CreateMotor(Motor6020C, 1, &motors_data1[0], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[1] = CreateMotor(Motor6020C, 2, &motors_data1[1], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[2] = CreateMotor(Motor6020C, 3, &motors_data1[2], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors1[3] = CreateMotor(Motor6020C, 4, &motors_data1[3], 4, DJI6020_PosSpdClose_mode2, DJI_motor_can_callback, CAN2);
}
#endif
/**
 * @brief 麦克纳姆轮底盘电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化麦克纳姆轮底盘的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_MECANUM_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{   
    #ifdef COMPILE_M_POWER
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    #else
    // 3508电机初始化 - 使用CAN2总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN2);
    #endif
}
#endif

/**
 * @brief 摆臂底盘电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化麦克纳姆轮底盘的3508电机，配置CAN通信和控制模式
 */
#ifdef ROBOTIC_SWING_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
    #ifdef COMPILE_M_POWER
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode2_MasterPower, DJI_motor_can_callback, CAN2);
    #else
    // 3508电机初始化 - 使用CAN2总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    #endif
    // // 3508电机初始化 - 使用CAN2总线
    // motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    // motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    // motors[2] = CreateMotor(Motor3508, 3, &motors_data[2], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    // motors[3] = CreateMotor(Motor3508, 4, &motors_data[3], 4, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
}
#endif

/**
 * @brief 摆臂抬升电机初始化函数
 * @param[in,out] motors 电机实例数组指针
 * @param[in,out] motors_data 电机控制数据数组指针
 * @details 初始化底盘的电机，配置CAN通信和控制模式
 */
#ifdef ROBOTIC_SWING_HOISTING
void Hoisting_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{

    motors[0] = CreateMotor(MotorGQ, 1, &motors_data[0], 4, GQ_Motor_Tqe_mode, GQ_motor_can_callback, CAN3);
    motors[1] = CreateMotor(MotorGQ, 2, &motors_data[1], 4, GQ_Motor_Tqe_mode, GQ_motor_can_callback, CAN3);
    motors[2] = CreateMotor(MotorGQ, 3, &motors_data[2], 4, GQ_Motor_Tqe_mode, GQ_motor_can_callback, CAN3);
    motors[3] = CreateMotor(MotorGQ, 4, &motors_data[3], 4, GQ_Motor_Tqe_mode, GQ_motor_can_callback, CAN3);
}
#endif

/**
 * @brief 靶车电机初始化函数
 * @param[in,out] motors 电机实例数组指针
 * @param[in,out] motors_data 电机控制数据数组指针
 * @details 初始化靶车的电机，配置CAN通信和控制模式
 */
#ifdef TARGET_CHASSIS
void Target_Chassis_Motor_Init(MotorInstance *motors_1, MotorControlData *motors_data_1, MotorInstance *motors_2, MotorControlData *motors_data_2)
{
    // 靶车电机初始化
    motors_1[0] = CreateMotor(Motor3508, 1, &motors_data_1[0], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    motors_1[1] = CreateMotor(Motor3508, 2, &motors_data_1[1], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);

    motors_2[0] = CreateMotor(MotorGQ, 2, &motors_data_2[0], 1, GQ_Motor_Current_mode, GQ_motor_can_callback, CAN2);  
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
    
    // YAW轴（偏航轴）电机
    motors[0] = CreateMotor(Motor6020C, 1, &motors_data[0], 2, DJI6020_current_mode, DJI_motor_can_callback, CAN3);
    // PITCH轴（俯仰轴）电机
    motors[1] = CreateMotor(Motor6020C, 2, &motors_data[1], 2, DJI6020_current_mode, DJI_motor_can_callback, CAN3);
}
#endif

#ifdef AUTO_SENTRY
void Gimbal_Motor_Init(MotorInstance *motors_1,MotorInstance *motors_2,MotorInstance *motors_3, MotorControlData *motors_data)
{
    // 6020电机初始化 - 使用CAN3总线
    
    // 小YAW轴（偏航轴）电机
    motors_1[0] = CreateMotor(Motor6020C, 1, &motors_data[0], 1, DJI6020_current_mode, DJI_motor_can_callback, CAN3);
    // 大YAW轴（偏航轴）电机
    motors_2[0] = CreateMotor(Motor4310, 2, &motors_data[1], 1, DM_Spd_mode, DM_motor_can_callback, CAN3);
    // PITCH轴（俯仰轴）电机
    motors_3[0] = CreateMotor(Motor4310, 1, &motors_data[2], 1, DM_Mit_mode, DM_motor_can_callback, CAN3);
}
#endif

/**
 * @brief 摆臂云台初始化函数
 * @param[in,out] motors 6020电机实例数组指针
 * @param[in,out] motors_data 6020电机控制数据数组指针
 * @details 初始化云台的6020电机（YAW轴和PITCH轴），配置CAN通信和控制模式
 */
#ifdef ROBOTIC_SWING_GIMBAL
void Gimbal_Yaw_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
    // YAW轴（偏航轴）电机，ID=5 CAN2总线
    motors[0] = CreateMotor(Motor6020V, 5, &motors_data[0], 1, DJI6020_current_mode, DJI_motor_can_callback, CAN2);

    Gimbal_6020[0] = motors[0];
}

void Gimbal_Pitch_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{
     // PITCH轴（俯仰轴）电机，ID=6 CAN1总线
    motors[0] = CreateMotor(Motor6020V, 6, &motors_data[0], 1, DJI6020_current_mode, DJI_motor_can_callback, CAN1);

    Gimbal_6020[1] = motors[0];
}
#endif

/**
 * @brief 无人机云台电机初始化函数
 * @param[in,out] motors 6020电机实例数组指针
 * @param[in,out] motors_data 6020电机控制数据数组指针
 * @details 初始化云台的6020电机（YAW轴和PITCH轴），配置CAN通信和控制模式
 */
#ifdef COMPILE_UAV_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorControlData *motors_data)
{   
    // 6020电机初始化 - 使用CAN3总线
    // YAW轴（偏航轴）电机，ID=7
    motors[0] = CreateMotor(Motor6020V, 7, &motors_data[0], 2, DJI6020_Voltage_mode, DJI_motor_can_callback, CAN1);
    // PITCH轴（俯仰轴）电机，ID=6
    motors[1] = CreateMotor(Motor6020V, 6, &motors_data[1], 2, DJI6020_Voltage_mode, DJI_motor_can_callback, CAN1);
}
#endif

/**
 * @brief 英雄云台电机初始化函数
 * @param[in,out] motors 6020电机实例数组指针
 * @param[in,out] motors_data 6020电机控制数据数组指针
 * @details 初始化云台的6020电机（YAW轴和PITCH轴），配置CAN通信和控制模式
 */
#ifdef COMPILE_HERO_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data)
{
    // YAW轴（偏航轴）电机，ID=1
    motors[0] = CreateMotor(Motor4310, 2, &motors_data[0], 1, DM_Mit_mode, DM_motor_can_callback, CAN2);
    // PITCH轴（俯仰轴）电机，ID=1
    motors1[0] = CreateMotor(Motor3508, 5, &motors_data[1], 1, DJI3508_Spd_mode, DJI_motor_can_callback, CAN3);
}
#endif

/**
 * @brief 机械臂电机初始化函数
 * @param[in,out] motors 4310电机实例数组指针
 * @param[in,out] motors1 2006电机实例数组指针
 * @param[in,out] motors_data 4310电机控制数据数组指针
 * @param[in,out] motors_data1 2006电机控制数据数组指针
 * @details 初始化机械臂的4310和2006电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_ARM
void Arm_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorInstance *motors2, 
    MotorControlData *motors_data, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 4310电机初始化 - 使用CAN1总线
    motors[0] = CreateMotor(Motor4310, 1, &motors_data[0], 2, DM_Psi_mode, DM_motor_can_callback, CAN1);
    motors[1] = CreateMotor(Motor4310, 3, &motors_data[1], 2, DM_Psi_mode, DM_motor_can_callback, CAN1);
    
    // 灵足电机初始化 - 使用CAN2总线
    motors1[0] = CreateMotor(MotorRS, 1, &motors_data1[0], 2, RS_Motor_Speed_mode, RS_motor_can_callback, CAN2);
    motors1[1] = CreateMotor(MotorRS, 2, &motors_data1[1], 2, RS_Motor_Speed_mode, RS_motor_can_callback, CAN2);

    // 高擎电机初始化 - 使用CAN3总线
    motors2[0] = CreateMotor(MotorGQ, 1, &motors_data2[0], 1, GQ_Motor_Pos_Vel_mode, GQ_motor_can_callback, CAN2);
}
#endif

/**
 * @brief 机械臂抬升电机初始化函数
 * @param[in,out] motors 4310电机实例数组指针
 * @param[in,out] motors1 2006电机实例数组指针
 * @param[in,out] motors_data 4310电机控制数据数组指针
 * @param[in,out] motors_data1 2006电机控制数据数组指针
 * @details 初始化机械臂的4310和2006电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_ARM_LIFT
void Arm_LIFT_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1)
{
    // 3508电机初始化 - 使用CAN3总线
    motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 1, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN3);
    
    // 2006电机初始化 - 使用CAN3总线
    motors1[0] = CreateMotor(Motor2006, 5, &motors_data1[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN3);
}
#endif

/**
 * @brief 步兵发射任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化发射任务的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN3总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 1, &motors_data1[0], 2, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN3);
    //右摩擦轮
    motors[1] = CreateMotor(Motor3508, 2, &motors_data1[1], 2, DJI3508_SpdClose_mode2, DJI_motor_can_callback, CAN3);
    // 2006电机初始化 - 使用CAN1总线
    motors2[0] = CreateMotor(Motor2006, 1, &motors_data2[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN1);
}
#endif

/**
 * @brief 摆臂发射任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化发射任务的3508电机，配置CAN通信和控制模式
 */
#ifdef ROBOTIC_SWING_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN1总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 2, &motors_data1[0], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    //右摩擦轮
    motors[1] = CreateMotor(Motor3508, 3, &motors_data1[1], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    // 2006电机初始化 - 使用CAN1总线
    motors2[0] = CreateMotor(Motor2006, 5, &motors_data2[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN1);
}
#endif

/**
 * @brief 无人机发射任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化发射任务的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_UAV_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN3总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 1, &motors_data1[0], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    //右摩擦轮
    motors[1] = CreateMotor(Motor3508, 2, &motors_data1[1], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    // 2006电机初始化 - 使用CAN3总线
    motors2[0] = CreateMotor(Motor2006, 3, &motors_data2[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN2);
}
#endif

/**
 * @brief 英雄发射任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化发射任务的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_HERO_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN3总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 1, &motors_data1[0], 3, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN3);
    //中摩擦轮
    motors[1] = CreateMotor(Motor3508, 2, &motors_data1[1], 3, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN3);
    //右摩擦轮
    motors[2] = CreateMotor(Motor3508, 3, &motors_data1[2], 3, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN3);
    //拨弹盘 - 使用CAN1总线
    motors1[0] = CreateMotor(Motor3508, 5, &motors_data2[0], 1, DJI3508_AngleSpdClose_mode2, DJI_motor_can_callback, CAN1);
}
#endif

/**
 * @brief 自动哨兵发射任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化发射任务的3508电机，配置CAN通信和控制模式
 */
#ifdef AUTO_SENTRY
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2)
{
    // 3508电机初始化 - 使用CAN1总线
    //左摩擦轮
    motors[0] = CreateMotor(Motor3508, 5, &motors_data1[0], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    //右摩擦轮
    motors[1] = CreateMotor(Motor3508, 6, &motors_data1[1], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN1);
    // 2006电机初始化 - 使用CAN1总线
    motors2[0] = CreateMotor(Motor2006, 7, &motors_data2[0], 1, DJI2006_SpdClose_mode, DJI_motor_can_callback, CAN1);
}
#endif

/**
 * @brief 测试任务电机初始化函数
 * @param[in,out] motors 3508电机实例数组指针
 * @param[in,out] motors_data 3508电机控制数据数组指针
 * @details 初始化测试任务的3508电机，配置CAN通信和控制模式
 */
#ifdef COMPILE_TEST
void Test_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3, MotorControlData *motors_data)
{
    
    motors[0] = CreateMotor(Motor6020C, 1, &motors_data[0], 2, DJI6020_current_mode, DJI_motor_can_callback, CAN3);
    motors[1] = CreateMotor(Motor6020V, 2, &motors_data[1], 2, DJI6020_Voltage_mode, DJI_motor_can_callback, CAN1);
    // 3508电机初始化 - 使用CAN2总线
    // motors[0] = CreateMotor(Motor3508, 1, &motors_data[0], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    // motors[1] = CreateMotor(Motor3508, 2, &motors_data[1], 2, DJI3508_SpdClose_mode, DJI_motor_can_callback, CAN2);
    motors2[0] = CreateMotor(Motor4310, 1, &motors_data[2], 3, DM_Spd_mode, DM_motor_can_callback, CAN2);
    motors2[1] = CreateMotor(Motor4310, 2, &motors_data[3], 3, DM_Spd_mode, DM_motor_can_callback, CAN2);
    motors2[2] = CreateMotor(Motor4310, 3, &motors_data[4], 3, DM_Spd_mode, DM_motor_can_callback, CAN2);
    motors3[0] = CreateMotor(MotorRS, 1, &motors_data[5], 2, RS_Motor_Pos_mode, RS_motor_can_callback, CAN5);
    motors3[1] = CreateMotor(MotorRS, 2, &motors_data[6], 2, RS_Motor_Pos_mode, RS_motor_can_callback, CAN5);
    //motors[0] = CreateMotor(MotorM15, 1, &motors_data[0], 4, DJI3508_Spd_mode, BM_motor_can_callback, CAN2);
    //motors[0] = CreateMotor(MotorRS, 1, &motors_data[0], 2, RS_Motor_Pos_mode, RS_motor_can_callback, CAN2);
    //motors[1] = CreateMotor(MotorRS, 2, &motors_data[1], 2, RS_Motor_Pos_mode, RS_motor_can_callback, CAN2);
    //  motors[0] = CreateMotor(MotorGQ, 1, &motors_data[0], 2, GQ_Motor_Pos_Vel_mode, GQ_motor_can_callback, CAN3);
    //  motors[1] = CreateMotor(MotorGQ, 2, &motors_data[1], 2, GQ_Motor_Pos_Vel_mode, GQ_motor_can_callback, CAN3);
    //motors[0] = CreateMotor(Motor4310, 1, &motors_data[0], 1, DM_Mit_mode, DM_motor_can_callback, CAN2);
    //motors[1] = CreateMotor(Motor4310, 2, &motors_data[1], 2, DM_Mit_mode, DM_motor_can_callback, CAN2);
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
    motor_data->vel = 0.0;                          // 速度初始化为0
    motor_data->kp = 0.0;                           // Kp参数初始化为0
    motor_data->kd = 0.0;                           // Kd参数初始化为0
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
            bsp_can1_register_callback(callback_func);       // 设置CAN1回调函数
            motor_data->hfdcan = &hfdcan1;              // 设置CAN1句柄
            break;
            
        case CAN2:
            bsp_can2_register_callback(callback_func);       // 注册CAN2回调函数
            motor_data->hfdcan = &hfdcan2;              // 设置CAN2句柄
            break;

        case CAN3:
            bsp_can3_register_callback(callback_func);       // 注册CAN3回调函数
            motor_data->hfdcan = &hfdcan3;              // 设置CAN3句柄
            break;

        case CAN4:
            bsp_can4_register_callback(callback_func);       // 注册CAN4(MCP2515 #1)回调函数
            motor_data->hfdcan = &hfdcan4_mcp2515;     // 设置虚拟CAN4句柄
            break;

        case CAN5:
            bsp_can5_register_callback(callback_func);       // 注册CAN5(MCP2515 #2)回调函数
            motor_data->hfdcan = &hfdcan5_mcp2515;     // 设置虚拟CAN5句柄
            break;

        case CAN6:
            bsp_can6_register_callback(callback_func);       // 注册CAN6(MCP2515 #3)回调函数
            motor_data->hfdcan = &hfdcan6_mcp2515;     // 设置虚拟CAN6句柄
            break;

        default:
            bsp_can2_register_callback(callback_func);       // 默认注册CAN2回调函数
            motor_data->hfdcan = &hfdcan2;              // 设置CAN2句柄
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
/**
 * @brief 抬升任务弱定义
 * @note 如果没有使用抬升功能，任务会自动删除
 */
__weak void arm_lift_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 摆臂抬升任务弱定义
 * @note 如果没有使用抬升功能，任务会自动删除
 */
__weak void hoisting_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief 小机械臂任务弱定义
 * @note 如果没有使用小机械臂，任务会自动删除
 */
__weak void lerobot_task(void *argument)
{
    vTaskDelete(NULL);
}
/**
 * @brief UI任务弱定义
 * @note 如果没有使用UI功能，任务会自动删除
 */
__weak void ui_task(void *argument)
{
    vTaskDelete(NULL);
}
