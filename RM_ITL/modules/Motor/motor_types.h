#ifndef __MOTOR_TYPES_H__
#define __MOTOR_TYPES_H__

#include "main.h"
#include "Pid.h"  // 包含PID结构体定义

struct MotorInstance;

// 定义电机反馈数据结构体
typedef struct 
{
    int p_int;
    int v_int;
    int t_int;
    float pos;  // 反馈位置
    float vel;  // 反馈角速度
    float tor;  // 反馈力矩
    float temp; // 反馈温度
    float current; // 反馈电流
} Motor_feedback;

// 电机控制数据结构体
typedef struct
{
    uint8_t id;                       // 电机ID
    uint8_t motor_enable;             // 电机使能状态，1为使能，0为未使能
    float pos;                        //电机反馈的位置
    float speed;                        //电机反馈线速度m/s
    float tor;                 
    float cur;
    float target_velocity;          // 目标角速度rpm
    float target_position;          // 目标位置
    float target_current;           // 目标电流
    float total_angle;                // 永久累加的总角度（度）
    float last_angle;                 // 上一次单圈角度（度）
    PID pid[3];                       // PID设置  
    Motor_feedback *feedback;         // 反馈数据指针
    FDCAN_HandleTypeDef *hfdcan;      // 使用的CAN
} MotorControlData;

// 定义电机类型
typedef enum
{
    Motor4310 = 0,
    Motor3508 = 0x200,
    Motor6020C = 0x204, //6020电流模式
    Motor6020V = 0x209, //6020电压模式
    Motor2006 = 0x1FF,
    MotorM15 = 0x32,
    MotorRS = 0x126,
    MotorGQ = 0x100
} MotorType;

// 定义CAN口
typedef enum 
{
    CAN1 = 0,
    CAN2 = 1,
    CAN3 = 2
} CAN_PORT;

// 定义电机状态类型
typedef enum 
{
    Motor_Disable = 0, //失能电机
    Motor_Enable = 1   //使能电机
} Motor_status;

// 定义电机注册结构体
typedef struct MotorInstance
{
    uint8_t motor_count;                                                 //同一个发送ID下的电机数量
    uint16_t type;                                                       //电机类型
    MotorControlData *motor_data;                                        //电机控制数据
    void (*motor_control)(struct MotorInstance *motors);                 // 电机控制函数
} MotorInstance;

#endif // __MOTOR_TYPES_H__
