#include "Torque_control.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

// ==================== Yaw轴动力学前馈 ====================
// τ = J·α + B·ω
// I = τ / K_t
// I_raw = I * (16384 / 3)
#define AMP_TO_RAW  (16384.0f / 3.0f)

float Yaw_Torque_Control(float omega, float alpha)
{
    float tau_inertia = Motor_J * alpha;        // 克服转动惯量
    float tau_viscous = Motor_B * omega;         // 克服粘滞阻尼
    float tau_ff = tau_inertia + tau_viscous;    // 总前馈扭矩
    float I_amp = tau_ff / Motor_Kt;             // 扭矩→电流(A)
    return I_amp * AMP_TO_RAW;                   // 电流→raw，可直接叠加到target_current
}

// ==================== Yaw轴扫频辨识 ====================
// 原理：给yaw电机施加线性调频正弦电流(chirp)，
//       同时记录电流、角速度、角加速度，通过串口发送到PC，
//       在PC上用最小二乘法拟合 τ = J·α + B·ω
//
// 模型：τ_applied = J · α + B · ω
//       τ_applied = I_raw * (3/16384) * K_t   (GM6020: 16384 raw = 3A)
//
// PC端最小二乘：
//   把每行数据看作 [α, ω] · [J, B]^T = τ
//   N个采样点组成超定方程 Y = X · θ
//   θ = (X^T X)^-1 X^T Y

// --- 扫频参数 ---
#define SWEEP_F_START       0.5f        // 起始频率 Hz
#define SWEEP_F_END         8.0f        // 终止频率 Hz
#define SWEEP_DURATION      10.0f       // 扫频总时长 s
#define SWEEP_AMPLITUDE     13000.0f    // 电流幅值 (raw, 范围±16384, 约2.4A)
#define SWEEP_DT            0.002f      // 控制周期 2ms
#define SWEEP_SAMPLE_SKIP   5           // 每5个周期采样一次 → 100Hz输出
// GM6020 raw电流→实际电流: 16384 raw = 3A
#define GM6020_RAW_TO_AMP   (3.0f / 16384.0f)

static float sweep_time = 0.0f;
uint8_t sweep_running = 0;
static uint16_t sweep_sample_cnt = 0;
static MotorInstance *sweep_motor = NULL;
float torque = 0.0f;
void Yaw_Sweep_Start(void)
{
    sweep_time = 0.0f;
    sweep_running = 1;
    sweep_sample_cnt = 0;
}

void Yaw_Sweep_Stop(void)
{
    sweep_running = 0;
    if (sweep_motor != NULL)
    {
        sweep_motor->motor_data->target_current = 0;
    }
}

uint8_t Yaw_Sweep_IsRunning(void)
{
    return sweep_running;
}

/**
 * @brief Yaw轴扫频测试（在云台任务中每2ms调用一次）
 * @param motor   yaw电机实例指针 (Gimbal_6020[0])
 * @param huart   用于数据输出的串口 (如 &huart1)
 * @param omega   当前yaw角速度 rad/s (INS.Gyro[Zt])
 * @param alpha   当前yaw角加速度 rad/s² (INS.GyroAccel[Zt])
 * @note  输出格式: torque(N·m), omega(rad/s), alpha(rad/s²)\n
 *        采样率100Hz，扫频10秒，共约1000行数据
 */
void Yaw_Sweep_Update(MotorInstance *motor, UART_HandleTypeDef *huart,
                      float omega, float alpha)
{
    if (!sweep_running || motor == NULL) return;

    sweep_motor = motor;

    // 扫频结束，电流归零
    if (sweep_time >= SWEEP_DURATION)
    {
        sweep_running = 0;
        motor->motor_data->target_current = 0;
        return;
    }

    // 线性调频 (chirp): 频率从 f_start 线性增加到 f_end
    // f(t) = f_start + (f_end - f_start) * t / T
    // φ(t) = 2π * [f_start*t + (f_end-f_start)*t² / (2T)]
    float phase = 2.0f * PI * (SWEEP_F_START * sweep_time
                + (SWEEP_F_END - SWEEP_F_START) * sweep_time * sweep_time
                / (2.0f * SWEEP_DURATION));
    float cmd = SWEEP_AMPLITUDE * sinf(phase);

    // 施加到电机（target_current 是 float，底层发送时会转 int16_t）
    motor->motor_data->target_current = cmd;

    // 降采样发送数据
    sweep_sample_cnt++;
    if (sweep_sample_cnt >= SWEEP_SAMPLE_SKIP)
    {
        sweep_sample_cnt = 0;
        torque = cmd * GM6020_RAW_TO_AMP * Motor_Kt;
        //usart_vofa_printf(huart, "%.4f,%.4f,%.4f\n", torque, omega, alpha);
    }

    sweep_time += SWEEP_DT;
}
