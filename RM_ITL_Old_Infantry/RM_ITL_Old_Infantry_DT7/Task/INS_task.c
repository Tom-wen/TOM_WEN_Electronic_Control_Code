
#include "INS_Task.h"
#include "cmsis_os.h"
#include "task.h"
#include "bsp_pwm.h"
#include "BMI088Driver.h"
#include "pid.h"
#include "tim.h"
#include <math.h>
#include "CAN_receive.h"
#include "kalman_filter.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_usart.h"
#include "USART_receive.h"
#include "bsp_led.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm) // pwm缁欏畾
INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};
BIM088_origin_data BIM088_data;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;//40
float speed = 0.0f;
float speed1 = 0.0f;
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);


INS_data_t INS_data;

/**
  * @brief          imu task, init bmi088, calculate the euler angle
  * @param[in]      argument: NULL
  * @retval         none
  */
void INS_task(void *argument) 
{
    
    osDelay(InsTask_INIT_TIME);

    while (BMI088_init()) 
    {
        osDelay(100);
    }
    INS_Init();
		
    while (1) 
    {
		INS_Task();
        INS_data.angle_yaw = INS.Yaw * PI / 180.0f;
        INS_data.angle_pitch = INS.Pitch * PI / 180.0f;
        INS_data.angle_roll = INS.Roll * PI / 180.0f;
        INS_data.wx = INS.Gyro[0];
        INS_data.wy = INS.Gyro[1];
        INS_data.wz = INS.Gyro[2];
        INS_data.ax = INS.Accel[0];
        INS_data.ay = INS.Accel[1];
        INS_data.az = INS.Accel[2];
        vTaskDelay(pdMS_TO_TICKS(2));
        

    }
}



void INS_Init(void)
{
    IMU_Param.scale[Xt] = 1;
    IMU_Param.scale[Yt] = 1;
    IMU_Param.scale[Zt] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // imu heat init
    PID_Init1(&TempCtrl, 800, 300, 0, 300, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    INS.AccelLPF = 0.0085;
    // 初始化yaw零飘估计参数（新增）
    INS.YawGyroBias = 0.0f;
    INS.StableCount = 0;
    INS.YawGyroSum = 0.0f;
    INS.BiasInitialized = 0;
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    static uint8_t sensor_init_count = 0;
    
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    
    // 检查dt是否有效
    if (dt <= 0.0f || dt > 0.1f || dt != dt) {
        count++;
        return;
    }
    
    t += dt;
    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(BIM088_data.gyro, BIM088_data.accel, &BIM088_data.temp);
        
        // 数据有效性检查 - 检查是否为NaN或超出合理范围
        uint8_t data_valid = 1;
        
        // 检查加速度计数据
        for (uint8_t i = 0; i < 3; i++) {
            if (BIM088_data.accel[i] != BIM088_data.accel[i] ||  // NaN检查
                fabsf(BIM088_data.accel[i]) > 100.0f) {        // 超出合理范围(100m/s?)
                data_valid = 0;
                break;
            }
        }
        
        // 检查陀螺仪数据
        if (data_valid) {
            for (uint8_t i = 0; i < 3; i++) {
                if (BIM088_data.gyro[i] != BIM088_data.gyro[i] ||  // NaN检查
                    fabsf(BIM088_data.gyro[i]) > 50.0f) {        // 超出合理范围(50rad/s)
                    data_valid = 0;
                    break;
                }
            }
        }
        
        // 如果数据无效，跳过本次更新
        if (!data_valid) {
            sensor_init_count++;
            if (sensor_init_count < 100) {  // 前100次允许数据不稳定
                count++;
                return;
            }
        } else {
            sensor_init_count = 0;
        }
        
        INS.Accel[Xt] = BIM088_data.accel[Xt];
        INS.Accel[Yt] = BIM088_data.accel[Yt];
        INS.Accel[Zt] = BIM088_data.accel[Zt];
        INS.Gyro[Xt] = BIM088_data.gyro[Xt];
        INS.Gyro[Yt] = BIM088_data.gyro[Yt];
        INS.Gyro[Zt] = BIM088_data.gyro[Zt];
        // =============== yaw零飘估计与补偿 ===============
        // 计算角速度和加速度模值
        float gyro_norm = sqrtf(INS.Gyro[Xt] * INS.Gyro[Xt] +
                                INS.Gyro[Yt] * INS.Gyro[Yt] +
                                INS.Gyro[Zt] * INS.Gyro[Zt]);
        float accel_norm = sqrtf(INS.Accel[Xt] * INS.Accel[Xt] +
                                 INS.Accel[Yt] * INS.Accel[Yt] +
                                 INS.Accel[Zt] * INS.Accel[Zt]);
        // 判断是否静止：角速度小且加速度接近重力
        if (gyro_norm < 0.05f && accel_norm > 9.3f && accel_norm < 10.3f)
        {
            // 累加yaw角速度
            INS.YawGyroSum += INS.Gyro[Zt];
            INS.StableCount++;
            if (INS.StableCount >= 100)
            {
                // 计算平均值作为零飘估计
                INS.YawGyroBias = INS.YawGyroBias * 0.95f +
                                  (INS.YawGyroSum / INS.StableCount) * 0.05f; // 低通滤波
                INS.BiasInitialized = 1;
                // 重置计数器
                INS.StableCount = 0;
                INS.YawGyroSum = 0.0f;
            }
        }
        else
        {
            // 运动状态，重置计数器
            INS.StableCount = 0;
            INS.YawGyroSum = 0.0f;
        }
        // 补偿yaw零飘（如果已初始化）
        if (INS.BiasInitialized)
        {
            INS.Gyro[Zt] -= INS.YawGyroBias;
        }
        // =============== yaw零飘估计与补偿结束 ===============
        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);
        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[Xt], INS.Accel[Zt]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Yt], INS.Accel[Zt]) * 180 / PI;
        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[Xt], INS.Gyro[Yt], INS.Gyro[Zt],
                                 INS.Accel[Xt], INS.Accel[Yt], INS.Accel[Zt], dt);
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);
        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                                    INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
        // 获取最终数据并检查NaN
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        
        // NaN检测与恢复
        if (INS.Yaw != INS.Yaw || INS.Pitch != INS.Pitch || INS.Roll != INS.Roll) {
            // 如果检测到NaN，重新初始化EKF
            IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
            INS.Yaw = 0.0f;
            INS.Pitch = 0.0f;
            INS.Roll = 0.0f;
        }
        
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
        INS.temp = BIM088_data.temp;
    }
    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }
    count++;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[Xt] = c_11 * gyro_temp[Xt] +
              c_12 * gyro_temp[Yt] +
              c_13 * gyro_temp[Zt];
    gyro[Yt] = c_21 * gyro_temp[Xt] +
              c_22 * gyro_temp[Yt] +
              c_23 * gyro_temp[Zt];
    gyro[Zt] = c_31 * gyro_temp[Xt] +
              c_32 * gyro_temp[Yt] +
              c_33 * gyro_temp[Zt];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[Xt] = c_11 * accel_temp[Xt] +
               c_12 * accel_temp[Yt] +
               c_13 * accel_temp[Zt];
    accel[Yt] = c_21 * accel_temp[Xt] +
               c_22 * accel_temp[Yt] +
               c_23 * accel_temp[Zt];
    accel[Zt] = c_31 * accel_temp[Xt] +
               c_32 * accel_temp[Yt] +
               c_33 * accel_temp[Zt];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 * 
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BIM088_data.temp, RefTemp);

    TIM_Set_PWM(&htim3, TIM_CHANNEL_4, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
