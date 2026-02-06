#ifndef __MAHONYAHRS_H
#define __MAHONYAHRS_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义 MahonyAHRS 结构体
typedef struct {
    float q[4];           // 四元数 [w, x, y, z]
    float sampleFreq;     // 采样频率 (Hz)
    float twoKp;          // 2 * 比例增益 (Kp)
    float twoKi;          // 2 * 积分增益 (Ki)
    float integralFBx;    // 积分误差项，由 Ki 缩放
    float integralFBy;
    float integralFBz;
} MahonyAHRS;

// 函数声明
void MahonyAHRS_init(MahonyAHRS* ahrs, float sampleFreq);
void MahonyAHRS_update(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRS_update_mag(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float* MahonyAHRS_get_q_data(MahonyAHRS* ahrs);
static float invSqrt(float x);

// 初始化函数实现
void MahonyAHRS_init(MahonyAHRS* ahrs, float sampleFreq) {
    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;
    ahrs->sampleFreq = sampleFreq;
    ahrs->twoKp = 2.0f * 0.5f;  // 2 * proportional gain (Kp)
    ahrs->twoKi = 2.0f * 0.0f;  // 2 * integral gain (Ki)
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;
}

// 获取四元数数据
float* MahonyAHRS_get_q_data(MahonyAHRS* ahrs) {
    return ahrs->q;
}

// 不带磁力计的更新函数
void MahonyAHRS_update(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 只有当加速度计测量有效时才计算反馈（避免加速度计归一化时出现 NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向和垂直于磁通量的向量
        halfvx = ahrs->q[1] * ahrs->q[3] - ahrs->q[0] * ahrs->q[2];
        halfvy = ahrs->q[0] * ahrs->q[1] + ahrs->q[2] * ahrs->q[3];
        halfvz = ahrs->q[0] * ahrs->q[0] - 0.5f + ahrs->q[3] * ahrs->q[3];

        // 误差是估计重力方向和测量重力方向叉积的和
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 如果启用了积分反馈，则计算并应用积分反馈
        if (ahrs->twoKi > 0.0f) {
            ahrs->integralFBx += ahrs->twoKi * halfex * (1.0f / ahrs->sampleFreq); // 积分误差乘以 Ki
            ahrs->integralFBy += ahrs->twoKi * halfey * (1.0f / ahrs->sampleFreq);
            ahrs->integralFBz += ahrs->twoKi * halfez * (1.0f / ahrs->sampleFreq);
            gx += ahrs->integralFBx; // 应用积分反馈
            gy += ahrs->integralFBy;
            gz += ahrs->integralFBz;
        } else {
            ahrs->integralFBx = 0.0f; // 防止积分饱和
            ahrs->integralFBy = 0.0f;
            ahrs->integralFBz = 0.0f;
        }

        // 应用比例反馈
        gx += ahrs->twoKp * halfex;
        gy += ahrs->twoKp * halfey;
        gz += ahrs->twoKp * halfez;
    }

    // 对四元数的变化率进行积分
    gx *= (0.5f * (1.0f / ahrs->sampleFreq)); // 预先乘以公共因子
    gy *= (0.5f * (1.0f / ahrs->sampleFreq));
    gz *= (0.5f * (1.0f / ahrs->sampleFreq));
    qa = ahrs->q[0];
    qb = ahrs->q[1];
    qc = ahrs->q[2];
    ahrs->q[0] += (-qb * gx - qc * gy - ahrs->q[3] * gz);
    ahrs->q[1] += (qa * gx + qc * gz - ahrs->q[3] * gy);
    ahrs->q[2] += (qa * gy - qb * gz + ahrs->q[3] * gx);
    ahrs->q[3] += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(ahrs->q[0] * ahrs->q[0] + ahrs->q[1] * ahrs->q[1] + ahrs->q[2] * ahrs->q[2] + ahrs->q[3] * ahrs->q[3]);
    ahrs->q[0] *= recipNorm;
    ahrs->q[1] *= recipNorm;
    ahrs->q[2] *= recipNorm;
    ahrs->q[3] *= recipNorm;
}

// 带磁力计的更新函数
void MahonyAHRS_update_mag(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果磁力计测量无效则使用 IMU 算法（避免磁力计归一化时出现 NaN）
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRS_update(ahrs, gx, gy, gz, ax, ay, az);
        return;
    }

    // 只有当加速度计测量有效时才计算反馈（避免加速度计归一化时出现 NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 归一化磁力计测量值
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量避免重复运算
        q0q0 = ahrs->q[0] * ahrs->q[0];
        q0q1 = ahrs->q[0] * ahrs->q[1];
        q0q2 = ahrs->q[0] * ahrs->q[2];
        q0q3 = ahrs->q[0] * ahrs->q[3];
        q1q1 = ahrs->q[1] * ahrs->q[1];
        q1q2 = ahrs->q[1] * ahrs->q[2];
        q1q3 = ahrs->q[1] * ahrs->q[3];
        q2q2 = ahrs->q[2] * ahrs->q[2];
        q2q3 = ahrs->q[2] * ahrs->q[3];
        q3q3 = ahrs->q[3] * ahrs->q[3];

        // 地磁场参考方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // 估计重力和磁场方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // 误差是估计方向与场向量测量方向叉积的和
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // 如果启用了积分反馈，则计算并应用积分反馈
        if (ahrs->twoKi > 0.0f) {
            ahrs->integralFBx += ahrs->twoKi * halfex * (1.0f / ahrs->sampleFreq); // 积分误差乘以 Ki
            ahrs->integralFBy += ahrs->twoKi * halfey * (1.0f / ahrs->sampleFreq);
            ahrs->integralFBz += ahrs->twoKi * halfez * (1.0f / ahrs->sampleFreq);
            gx += ahrs->integralFBx; // 应用积分反馈
            gy += ahrs->integralFBy;
            gz += ahrs->integralFBz;
        } else {
            ahrs->integralFBx = 0.0f; // 防止积分饱和
            ahrs->integralFBy = 0.0f;
            ahrs->integralFBz = 0.0f;
        }

        // 应用比例反馈
        gx += ahrs->twoKp * halfex;
        gy += ahrs->twoKp * halfey;
        gz += ahrs->twoKp * halfez;
    }

    // 对四元数的变化率进行积分
    gx *= (0.5f * (1.0f / ahrs->sampleFreq)); // 预先乘以公共因子
    gy *= (0.5f * (1.0f / ahrs->sampleFreq));
    gz *= (0.5f * (1.0f / ahrs->sampleFreq));
    qa = ahrs->q[0];
    qb = ahrs->q[1];
    qc = ahrs->q[2];
    ahrs->q[0] += (-qb * gx - qc * gy - ahrs->q[3] * gz);
    ahrs->q[1] += (qa * gx + qc * gz - ahrs->q[3] * gy);
    ahrs->q[2] += (qa * gy - qb * gz + ahrs->q[3] * gx);
    ahrs->q[3] += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(ahrs->q[0] * ahrs->q[0] + ahrs->q[1] * ahrs->q[1] + ahrs->q[2] * ahrs->q[2] + ahrs->q[3] * ahrs->q[3]);
    ahrs->q[0] *= recipNorm;
    ahrs->q[1] *= recipNorm;
    ahrs->q[2] *= recipNorm;
    ahrs->q[3] *= recipNorm;
}

// 快速平方根倒数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

#ifdef __cplusplus
}
#endif

#endif /* __MAHONYAHRS_H */