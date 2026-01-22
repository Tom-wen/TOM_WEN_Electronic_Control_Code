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
  FuzzyPID_struct_t gimbal_motor_relative_angle_fuzzy_pid;  // 模糊PID - 位置环
  FuzzyPID_struct_t gimbal_motor_gyro_fuzzy_pid;             // 模糊PID - 速度环
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

  float Feedforward;    //前馈控制

  float radar_add;  //雷达发送的增量角度

} gimbal_motor_t;

extern gimbal_motor_t gimbal_motor_t_yaw;
extern gimbal_motor_t gimbal_motor_t_pitch;
extern CAN_TxHeaderTypeDef tx_header;

void gimbal_yaw_init(void);
void gimbal_pitch_init(void);
void gimbal_yaw_PID_Calc(gimbal_motor_t* gimbal);
void gimbal_pitch_PID_Calc(gimbal_motor_t* gimbal);
void CAN_yaw_cmd_gimbal(gimbal_motor_t* gimbal);
void CAN_pitch_cmd_gimbal(gimbal_motor_t* gimbal);
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id);
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id);
void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms);
void gimbal_set_control_init(void);
void gimbal_set_control(void);
void motor_read(CAN_HandleTypeDef *hcan, uint8_t id);
void motor_control_cur(CAN_HandleTypeDef *hcan, uint8_t id, int16_t cur);


#define ANGLE_T 8191.0f
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
    //M6020电机测量数据结构体
  typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int round;
  float angle;
} motor_measure_t;
extern motor_measure_t M6020_motor_measure;
void M6020_send(uint16_t data);
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
void update_pitch(gimbal_motor_t* gimbal,motor_measure_t* M6020_motor_measure_t,CAN_RxHeaderTypeDef* rx_header);
void update_yaw(gimbal_motor_t* gimbal,uint8_t rx_data[24],CAN_RxHeaderTypeDef* rx_header);



//pid参数
#define yaw_absolute_angle_kp 0.0f
#define yaw_absolute_angle_ki 0.0f
#define yaw_absolute_angle_kd 0.0f
#define yaw_absolute_angle_imax 1000.0f
#define yaw_absolute_angle_out_max 1000.0f


#define yaw_gyro_kp 5.05f
#define yaw_gyro_ki 0.0f
#define yaw_gyro_kd 3.5f
#define yaw_gyro_imax 1500.0f
#define yaw_gyro_out_max 4200.0f

#define yaw_relative_angle_kp 55.0f
#define yaw_relative_angle_ki 0.05f
#define yaw_relative_angle_kd 25.0f
#define yaw_relative_angle_imax 1000.0f
#define yaw_relative_angle_out_max 3000.0f




#define pitch_absolute_angle_kp 0.0f
#define pitch_absolute_angle_ki 0.0f
#define pitch_absolute_angle_kd 0.0f
#define pitch_absolute_angle_imax 1000.0f
#define pitch_absolute_angle_out_max 1000.0f


#define pitch_gyro_kp 90.350f
#define pitch_gyro_ki 5.828f
#define pitch_gyro_kd 10.0f
#define pitch_gyro_imax 200.0f
#define pitch_gyro_out_max 3500.0f

#define pitch_relative_angle_kp 1000.0f
#define pitch_relative_angle_ki 2.2f
#define pitch_relative_angle_kd 10.0f
#define pitch_relative_angle_imax 4000.0f
#define pitch_relative_angle_out_max 3400.0f



//限幅
#define yaw_max_relative_angle 0.48f
#define yaw_min_relative_angle 0.52f
#define pitch_max_relative_angle 3.0f
#define pitch_min_relative_angle -3.0f

// 模糊PID参数定义
// Yaw轴模糊PID参数
#define yaw_fuzzy_relative_angle_kp 3585.5f
#define yaw_fuzzy_relative_angle_ki 1.6969f
#define yaw_fuzzy_relative_angle_kd 5.999f
#define yaw_fuzzy_relative_angle_out_max 5950.0f
#define yaw_fuzzy_relative_angle_e_max 0.5f
#define yaw_fuzzy_relative_angle_e_min -0.5f
#define yaw_fuzzy_relative_angle_ec_max 0.1f
#define yaw_fuzzy_relative_angle_ec_min -0.1f
#define yaw_fuzzy_relative_angle_kp_max 500.0f
#define yaw_fuzzy_relative_angle_kp_min -500.0f
#define yaw_fuzzy_relative_angle_ki_max 0.5f
#define yaw_fuzzy_relative_angle_ki_min -0.5f
#define yaw_fuzzy_relative_angle_kd_max 2.0f
#define yaw_fuzzy_relative_angle_kd_min -2.0f

#define yaw_fuzzy_gyro_kp 6.398f
#define yaw_fuzzy_gyro_ki 0.0f
#define yaw_fuzzy_gyro_kd 1.218f
#define yaw_fuzzy_gyro_out_max 3070.0f
#define yaw_fuzzy_gyro_e_max 10.0f
#define yaw_fuzzy_gyro_e_min -10.0f
#define yaw_fuzzy_gyro_ec_max 5.0f
#define yaw_fuzzy_gyro_ec_min -5.0f
#define yaw_fuzzy_gyro_kp_max 1.0f
#define yaw_fuzzy_gyro_kp_min -1.0f
#define yaw_fuzzy_gyro_ki_max 0.1f
#define yaw_fuzzy_gyro_ki_min -0.1f
#define yaw_fuzzy_gyro_kd_max 0.5f
#define yaw_fuzzy_gyro_kd_min -0.5f

// Pitch轴模糊PID参数
#define pitch_fuzzy_relative_angle_kp 2550.0f
#define pitch_fuzzy_relative_angle_ki 0.006f
#define pitch_fuzzy_relative_angle_kd 1.65f
#define pitch_fuzzy_relative_angle_out_max 3400.0f
#define pitch_fuzzy_relative_angle_e_max 0.5f
#define pitch_fuzzy_relative_angle_e_min -0.5f
#define pitch_fuzzy_relative_angle_ec_max 0.1f
#define pitch_fuzzy_relative_angle_ec_min -0.1f
#define pitch_fuzzy_relative_angle_kp_max 500.0f
#define pitch_fuzzy_relative_angle_kp_min -500.0f
#define pitch_fuzzy_relative_angle_ki_max 0.5f
#define pitch_fuzzy_relative_angle_ki_min -0.5f
#define pitch_fuzzy_relative_angle_kd_max 2.0f
#define pitch_fuzzy_relative_angle_kd_min -2.0f

#define pitch_fuzzy_gyro_kp 3.350f
#define pitch_fuzzy_gyro_ki 0.0f
#define pitch_fuzzy_gyro_kd 1.85f
#define pitch_fuzzy_gyro_out_max 5500.0f
#define pitch_fuzzy_gyro_e_max 10.0f
#define pitch_fuzzy_gyro_e_min -10.0f
#define pitch_fuzzy_gyro_ec_max 5.0f
#define pitch_fuzzy_gyro_ec_min -5.0f
#define pitch_fuzzy_gyro_kp_max 1.0f
#define pitch_fuzzy_gyro_kp_min -1.0f
#define pitch_fuzzy_gyro_ki_max 0.1f
#define pitch_fuzzy_gyro_ki_min -0.1f
#define pitch_fuzzy_gyro_kd_max 0.5f
#define pitch_fuzzy_gyro_kd_min -0.5f





#ifdef __cplusplus
}
#endif

#endif
