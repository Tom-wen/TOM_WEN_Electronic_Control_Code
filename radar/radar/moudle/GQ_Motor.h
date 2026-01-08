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
#include "stdio.h"
#include "math.h"




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
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad     // 编码器角度
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float current_set;
  float given_current;

  float radar_add;  //雷达发送的增量角度

} gimbal_motor_t;

extern gimbal_motor_t gimbal_motor_t_yaw;
extern gimbal_motor_t gimbal_motor_t_pitch;
extern CAN_TxHeaderTypeDef tx_header;

void gimbal_yaw_init(void);
void gimbal_pitch_init(void);
void update(gimbal_motor_t* gimbal,uint8_t rx_data[24],CAN_RxHeaderTypeDef* rx_header);
void gimbal_PID_Calc(gimbal_motor_t* gimbal);
void CAN_cmd_gimbal(gimbal_motor_t* gimbal);
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id);
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id);
void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms);
void gimbal_set_control_init(void);
void gimbal_set_control(void);
void motor_read(CAN_HandleTypeDef *hcan, uint8_t id);



//pid参数
#define yaw_absolute_angle_kp 0.0f
#define yaw_absolute_angle_ki 0.0f
#define yaw_absolute_angle_kd 0.0f
#define yaw_absolute_angle_imax 1000.0f
#define yaw_absolute_angle_out_max 1000.0f


#define yaw_gyro_kp 6.398f
#define yaw_gyro_ki 0.0f
#define yaw_gyro_kd 1.218f
#define yaw_gyro_imax 1000.0f
#define yaw_gyro_out_max 3070.0f

#define yaw_relative_angle_kp 3585.5f
#define yaw_relative_angle_ki 1.6969f
#define yaw_relative_angle_kd 5.999f
#define yaw_relative_angle_imax 250.0f
#define yaw_relative_angle_out_max 5950.0f


#define pitch_absolute_angle_kp 0.0f
#define pitch_absolute_angle_ki 0.0f
#define pitch_absolute_angle_kd 0.0f
#define pitch_absolute_angle_imax 1000.0f
#define pitch_absolute_angle_out_max 1000.0f


#define pitch_gyro_kp 6.398f
#define pitch_gyro_ki 0.0f
#define pitch_gyro_kd 1.218f
#define pitch_gyro_imax 1000.0f
#define pitch_gyro_out_max 3070.0f

#define pitch_relative_angle_kp 3585.5f
#define pitch_relative_angle_ki 1.6969f
#define pitch_relative_angle_kd 5.999f
#define pitch_relative_angle_imax 250.0f
#define pitch_relative_angle_out_max 5950.0f


//限幅
#define yaw_max_relative_angle 0.48f
#define yaw_min_relative_angle 0.52f
#define pitch_max_relative_angle 3.0f
#define pitch_min_relative_angle -3.0f

#ifdef __cplusplus
}
#endif

#endif