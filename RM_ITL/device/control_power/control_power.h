//
// Created by 21341 on 25-11-18.
//

#ifndef __CONTROL_POWER_H__
#define __CONTROL_POWER_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "bsp_can.h"

#define M_Too_Small_AllErrors 500.0          //总误差阈值
// #define M_Enable_PowerCompensation        //使能功率补偿
#define M_Power_Compensation_Alpha 0.05      //功率补偿参数  
#define M_Motor_ReservedPower_Border 54.0    //电机保留功率阈值 
#define M_PerMotor_ReservedPower 8.0         //每个电机保留功率

// 功率控制参数
#define POWER_RESERVE_RATE      0.85f       // 预留百分比，最终限制功率 = 输入值 * 此比例
#define MOTOR_STATIC_POWER      8.0f        // 电机静态功率（用于衰减计算的分母保护）
#define POWER_HIGH_THRESHOLD    50.0f       // 高功率阈值，超过此值时使用静态功率保护


typedef struct 
{
    float pm_voltage;
    float pm_current;
    float pm_power;
} pm_power;


// MotorPower 结构体定义
typedef struct {
    float k0, k1, k2, k3, k4, k5, Current_Conversion;
    float feedback_power;
    float predict_not_limit_power;
    float predict_power;
    //
    float power_limit; //单个电机目标限制功率
    float cal_current;
    float cal_speed;
    float speed_error;
    float ratio;
    //从外界传入的值
    float target_speed;
    float current_speed;
    float current_current;
    float desired_current;
    //向外界输出的系数
    float decay_number; //衰减系数
} MotorPower;

typedef struct {
    MotorPower* chassis_motors[4];
    float motor_errors[4];
} ChassisPowerManager;

typedef enum {
    E_disabled_negative,
    E_enable_negative
} E_CalMotorPower_Negative_Status_Type;

typedef enum {
    motor3508,
    motor6020
} power_motor_type;

typedef enum {
    E_disabled_predict,
    E_enable_predict,
    E_enable_not_limit_predict
} E_Predict_Status_Type;

typedef enum {
    E_control_hero,
    E_control_infantry,
    E_control_engineer,
    E_control_sentry,
} E_Control_Target_Type;

typedef struct {
    float k0, k1, k2, k3, k4, k5;
    float real_current_conversion;
} motor_power_init_t;

typedef struct {
    uint8_t battery_power;
    uint8_t limit_power;
    uint8_t capacitor_level;
    float capacitor_voltage;
    float capacitor_current;
} super_cap_data;

//外部声明
extern pm_power power_receive_data;
extern MotorPower chassis_3508_motor[4];
extern float motor3508_current_power[4];
extern float chassis_total_power;
extern super_cap_data super_cap;
// 函数声明
void motor_power_init(MotorPower* motor, const motor_power_init_t* init_data);
void Chassis_Power_Init(void);
float Chassis_3508_Get_Limited_Number(uint8_t motor_id);
float Chassis_6020_Get_Limited_Number(uint8_t motor_id);
float motor_power_get_real_current(MotorPower *motor, uint8_t motor_id);
float Chassis_3508_Actual_Power_Calculate(MotorPower *chassis_3508_motor, uint8_t motor3508_number);
float Chassis_6020_Actual_Power_Calculate(MotorPower *chassis_6020_motor, uint8_t motor6020_number);
float Power_Control(FDCAN_HandleTypeDef *hfdcan, float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number);
float motor_power_clac(MotorPower *motor, uint8_t motor_id);
void power_data_callback(CANRxData *Rx_data, CAN_PORT can_port);
void super_cap_callback(CANRxData *Rx_data, CAN_PORT can_port);
void super_cap_limitpower_sent(FDCAN_HandleTypeDef *hfdcan, float power);

// 裁判系统接口（需要用户实现）
float get_referee_chassis_power_limit(void);
float get_referee_buffer_energy(void);

#endif
