#ifndef __INIT_H__
#define __INIT_H__

#include "main.h"
#include <stdlib.h>
#include "motor_types.h"  // 首先包含基础类型定义
//bsp初始化
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
//modules初始化
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "RS_Motor.h"
#include "GQ_Motor.h"
#include "motor_types.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include "Pid.h"
#include "PID_Test.h"
#include "Lowpass.h"
//app初始化
#include "mecanum_chassis.h"
#include "ins.h"
#include "detect.h"
#include "referee.h"
#include "ui.h"
//device
#include "control_power.h"
#include "Circular_buffer.h"
//选择板子类型
#define gimbal_board    //云台控制板

//控制app层是否应用
//底盘
#define COMPILE_MECANUM_CHASSIS  //麦克纳姆轮底盘
//云台
#define COMPILE_GIMBAL
#define COMPILE_SHOOT
#define COMPILE_REFEREE
//功率控制
#define COMPILE_POWER

#define Radius  45    //底盘中心到6020中心的半径

void Robot_Init(void);
void Motor_Init();
#ifdef COMPILE_MECANUM_CHASSIS
void Chassis_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//初始化底盘电机结构体声明
extern MotorInstance Chassis_3508[4];
#endif
#ifdef COMPILE_GIMBAL
void Gimbal_Motor_Init(MotorInstance *motors, MotorControlData *motors_data);
//云台
extern MotorInstance Gimbal_6020[2]; 
#endif
#ifdef COMPILE_SHOOT
void Shoot_Motor_Init(MotorInstance *motors, MotorInstance *motors2, MotorControlData *motors_data1, MotorControlData *motors_data2);
//发射机构
extern MotorInstance Shoot_3508[2];
extern MotorInstance Trigger_2006[1]; 
#endif
MotorInstance CreateMotor(uint16_t type, uint8_t id, MotorControlData *motor_data, uint8_t motor_count,
                       void (*control_func)(MotorInstance *motors),                
                       void (*callback_func)(CANRxData *rx_data, CAN_PORT can_port), CAN_PORT can_port);

#endif
