/**
 ******************************************************************************
 * @file    lqr_basic.h
 * @brief   LQR状态反馈控制器
 *
 *          LQR_BASIC实现一个2入4出的线性二次型调节器。
 *          状态向量 x = [phi, phi_dot, theta, theta_dot]'
 *          控制输出 u = [tor_phi, tor_theta]'
 *          控制律: u = -K * x  (带输出限幅)
 *          K矩阵通过MATLAB lqr()函数离线计算得到。
 ******************************************************************************
 */

#ifndef LQR_BASIC_H
#define LQR_BASIC_H

#include "main.h"
#include "config.h"
#include <math.h>

typedef struct
{
    // ==================== 初始化参数 ====================
    // LQR增益矩阵 K(2x4)
    // K[0][0~3]: phi/phi_dot/theta/theta_dot -> tor_phi 的增益
    // K[1][0~3]: phi/phi_dot/theta/theta_dot -> tor_theta 的增益
    float K[2][4];
    // 输出力矩限幅 (Nm)
    float phi_limit;
    float theta_limit;

    // ==================== 读变量 ====================
    // 状态输入
    float phi;         // 当前phi轴位置 (rad)
    float phi_dot;     // 当前phi轴角速度 (rad/s)
    float theta;       // 当前theta轴位置 (rad)
    float theta_dot;   // 当前theta轴角速度 (rad/s)
    // 控制输出
    float tor_phi;     // phi轴控制力矩 (Nm)
    float tor_theta;   // theta轴控制力矩 (Nm)
} LqrBasic;

void LqrBasic_Init(LqrBasic *lqr, const float K[2][4], float phi_limit, float theta_limit);
void LqrBasic_Calc(LqrBasic *lqr, float phi, float phi_dot, float theta, float theta_dot);
void LqrBasic_SetK(LqrBasic *lqr, const float K[2][4]);
void LqrBasic_Clear(LqrBasic *lqr);

#endif
