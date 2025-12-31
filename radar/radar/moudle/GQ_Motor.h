#ifndef __GQ_MOTOR_H_
#define __GQ_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"
#include "PID.h"
#include "string.h"
#include "can.h"



typedef struct
{
    uint32_t id;
    float position;
    float velocity;
    float torque;
} motor_state_s;

typedef struct
{
    union
    {
        motor_state_s motor;
        uint8_t data[16];
    }motor_data;
} motor_state_t;


typedef struct
{
  motor_state_t gimbal_motor_measure;
  PID_struct_t gimbal_motor_absolute_angle_pid;
  PID_struct_t gimbal_motor_relative_angle_pid;
  PID_struct_t gimbal_motor_gyro_pid;
  float offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad     // ±àÂëÆ÷½Ç¶È
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  float given_current;
} gimbal_motor_t;

extern gimbal_motor_t gimbal_motor_t_yaw;
extern gimbal_motor_t gimbal_motor_t_pitch;
extern CAN_TxHeaderTypeDef tx_header;

void gimbal_yaw_init(void);
void update(gimbal_motor_t* gimbal,uint8_t rx_data[24],CAN_RxHeaderTypeDef* rx_header);
void gimbal_PID_Calc(gimbal_motor_t* gimbal);
void CAN_cmd_gimbal(gimbal_motor_t* gimbal);
void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms);


#define yaw_absolute_angle_kp 0.0f
#define yaw_absolute_angle_ki 0.0f
#define yaw_absolute_angle_kd 0.0f
#define yaw_absolute_angle_imax 2000.0f
#define yaw_absolute_angle_out_max 2000.0f
#define yaw_gyro_kp 0.40f
#define yaw_gyro_ki 0.0f
#define yaw_gyro_kd 0.05f
#define yaw_gyro_imax 2000.0f
#define yaw_gyro_out_max 2000.0f
#define yaw_relative_angle_kp 2000.0f
#define yaw_relative_angle_ki 0.0f
#define yaw_relative_angle_kd 0.05f
#define yaw_relative_angle_imax 2000.0f
#define yaw_relative_angle_out_max 2000.0f




#ifdef __cplusplus
}
#endif

#endif