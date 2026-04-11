#ifndef __INS_H__
#define __INS_H__

#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "bsp_pwm.h"

#define Xt 0
#define Yt 1
#define Zt 2

#define INS_TASK_PERIOD 1

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

extern INS_t INS;
extern float target_angle;

void ins_task(void *argument);
void INS_Init(void);
void IMU_Temperature_Ctrl();
void INS_Task(void);
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
