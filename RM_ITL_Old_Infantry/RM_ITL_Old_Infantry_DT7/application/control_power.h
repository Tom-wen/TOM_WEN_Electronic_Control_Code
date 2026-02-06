//
// Created by 21341 on 25-11-18.
//

#ifndef __CONTROL_POWER_H__
#define __CONTROL_POWER_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define M_Too_Small_AllErrors 500.0          //总误差阈�?
// #define M_Enable_PowerCompensation        //使能功率补偿
#define M_Power_Compensation_Alpha 0.05      //功率补偿参数  
#define M_Motor_ReservedPower_Border 54.0    //电机保留功率阈�? 
#define M_PerMotor_ReservedPower 8.0         //每个电机保留功率

// MotorPower 结构体定�?
typedef struct {
    double k0, k1, k2, k3, k4, k5, Current_Conversion;
    double feedback_power;
    double predict_not_limit_power;
    double predict_power;
    //
    double power_limit;
    double cal_current;
    double cal_speed;
    //从外界传入的�?
    double current_speed;
    double current_current;
    double desired_current;
    //向外界输出的系数
    double decay_number; //衰减系数
} MotorPower;

typedef struct {
    MotorPower* chassis_motors[4];
    double motor_errors[4];
} ChassisPowerManager;

typedef enum {
    E_disabled_negative,
    E_enable_negative
} E_CalMotorPower_Negative_Status_Type;

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
    double k0, k1, k2, k3, k4, k5;
    double real_current_conversion;
} motor_power_init_t;

// 函数声明
void Chassis_Power_Init(void);
void Chassis_Power_Total_Control(float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number);
float Chassis_3508_Actual_Power_Calculate(MotorPower *chassis_3508_motor, uint8_t motor3508_number);
float Chassis_6020_Actual_Power_Calculate(MotorPower *chassis_6020_motor, uint8_t motor6020_number);
float Chassis_6020_Get_Limited_Number(uint8_t motor_id);
float Chassis_3508_Get_Limited_Number(uint8_t motor_id);
void Chassis_Power_Total_Allocate(float total_power_limit, float need_power_limit[2], uint8_t motor3508_number, uint8_t motor6020_number);
void Total_3508motor_Power_Control(float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number);
void Total_6020motor_Power_Control(float total_power_limit, uint8_t motor6020_number, uint8_t motor3508_number);
double motor_power_update(MotorPower* motor, E_Predict_Status_Type predict_status, 
                         E_CalMotorPower_Negative_Status_Type negative_status);
double motor_power_get_real_current(const MotorPower* motor, double current);
double chassis_power_manager_get_total_predict_not_limit_power(const ChassisPowerManager* chassis);
double chassis_power_manager_get_total_power_limit(const ChassisPowerManager* chassis);
double calculate_current_from_power(MotorPower* motor, double power, double speed);
double motor_power_limiter(MotorPower *chassis_motor, uint8_t number);
double max(double a, double b);
#endif
