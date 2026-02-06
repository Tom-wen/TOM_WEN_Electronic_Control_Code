/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       IMU_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define Xt 0
#define Yt 1
#define Zt 2




#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define TEMPERATURE_PID_KP 1600.0f // 温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    // 温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    // 温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f // 温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  // 温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 // mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define InsTask_INIT_TIME 7 // 任务开始初期 delay 一段时间



typedef struct
{
  float angle_yaw;
  float angle_pitch;
  float angle_roll;

  float wx;
  float wy;
  float wz;

  float ax;
  float ay;
  float az;
} INS_data_t;

typedef struct
{
    float q[4]; // 四元数估计值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度
    float AccelLPF; // 加速度低通滤波系数
    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];
    float atanxz;
    float atanyz;
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    //温度
    float temp;
    // yaw轴零飘估计（新增）
    float YawGyroBias;          // yaw轴零飘估计值 (rad/s)
    uint16_t StableCount;       // 静止计数器
    float YawGyroSum;           // 静止时yaw角速度累加和
    uint8_t BiasInitialized;    // 零飘是否已初始化
} INS_t;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;


extern INS_data_t INS_data;
extern INS_t INS;

void INS_Init(void);
void IMU_Temperature_Ctrl();
void INS_Task(void);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#ifdef __cplusplus
}
#endif

#endif
