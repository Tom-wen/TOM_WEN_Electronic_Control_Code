#ifndef __INIT_H__
#define __INIT_H__

//系统初始化
#include "main.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

//bsp初始化
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_led.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_usb.h"
//modules初始化
#include "BMI088driver.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "GQ_Motor.h"
#include "RS_Motor.h"
#include "FT_Motor.h"
#include "PID_Test.h"
#include "remote_control.h"
#include "motor_types.h"  // 首先包含基础类型定义
//device
#include "control_power.h"
#include "Circular_buffer.h"
#include "Pid.h"
#include "Lowpass.h"
#include "CRC.h"
#include "Hot_control.h"
#include "Signal.h"
#include "yaw_auto_lqr_eso_controller.h"

void Robot_Init(void);
void RTOS_Init(void);
void Motor_Init();
#ifdef COMPILE_AGV_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
extern MotorInstance Chassis_6020[4];
#endif
#ifdef COMPILE_HERO_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
extern MotorInstance Chassis_6020[4];
#endif
#ifdef COMPILE_M_HERO_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
#endif
#ifdef AUTO_SENTRY
void Chassis_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
extern MotorInstance Chassis_6020[4];
#endif
#ifdef COMPILE_MECANUM_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
#endif
#ifdef ROBOTIC_SWING_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
#endif
#ifdef ROBOTIC_SWING_HOISTING
void Hoisting_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//初始化抬升电机结构体声明
extern MotorInstance Hoisting_5047[4];
#endif
#ifdef TARGET_CHASSIS
void Target_Chassis_Motor_Init(MotorInstance *motors_1, MotorControlData *motors_data_1, MotorInstance *motors_2, MotorControlData *motors_data_2);
//初始化电机结构体声明
extern MotorInstance Chassis_3508[2];
extern MotorInstance Gimbal_5047[1];
#endif
#ifdef COMPILE_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_6020[2]; 
#endif
#ifdef ROBOTIC_SWING_GIMBAL
void Gimbal_Yaw_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
void Gimbal_Pitch_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_6020[2]; 
#endif
#ifdef AUTO_SENTRY
void Gimbal_Motor_Init(MotorInstance *motors_1,MotorInstance *motors_2,MotorInstance *motors_3, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_Small_Yaw_6020[1]; 
extern MotorInstance Gimbal_Big_Yaw_4310[1];
extern MotorInstance Gimbal_Pitch_4310[1];
#endif
#ifdef COMPILE_UAV_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_6020[2];  
#endif
#ifdef COMPILE_HERO_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_4310[1]; 
extern MotorInstance Gimbal_3508[1]; 
#endif
#ifdef COMPILE_ARM
void Arm_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorInstance *motors2, MotorControlData *motors_data, MotorControlData *motors_data1, MotorControlData *motors_data2);
//机械臂
extern MotorInstance Arm_4310[2];
extern MotorInstance Arm_EL05[2];
extern MotorInstance Arm_6036[1];
#endif
#ifdef COMPILE_ARM_LIFT
void Arm_LIFT_Motor_Init(MotorInstance *motors, MotorInstance *motors1, MotorControlData *motors_data, MotorControlData *motors_data1);
//机械臂
extern MotorInstance Arm_3508[1];
extern MotorInstance Arm_2006[1];
#endif
#ifdef COMPILE_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[2];
extern MotorInstance Trigger_2006[1]; 
#endif
#ifdef ROBOTIC_SWING_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[2];
extern MotorInstance Trigger_2006[1]; 
#endif
#ifdef COMPILE_UAV_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[2];
extern MotorInstance Trigger_2006[1]; 
#endif
#ifdef COMPILE_HERO_SHOOT
//发射机构
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[3];
extern MotorInstance Trigger_3508[1]; 
#endif
#ifdef AUTO_SENTRY
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[2];
extern MotorInstance Trigger_2006[1]; 
#endif
#ifdef COMPILE_TEST
void Test_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3, MotorControlData *motors_data);
//测试任务
extern MotorInstance test_3508[2];
extern MotorInstance test_4310[3];
extern MotorInstance test_EL05[2];
#endif 
MotorInstance CreateMotor(uint16_t type, uint8_t id, MotorControlData *motor_data, uint8_t motor_count,
                       void (*control_func)(MotorInstance *motors),                
                       void (*callback_func)(CANRxData *rx_data, CAN_PORT can_port), CAN_PORT can_port);

#endif
