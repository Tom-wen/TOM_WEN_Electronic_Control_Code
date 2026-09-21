#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdint.h>

/* 信号发生器参数结构体 */
typedef struct 
{
    float amplitude;    // 幅值
    float frequency;    // 频率 (Hz)
    float offset;       // 直流偏置
    float phase;        // 相位 (rad)
    float time;         // 内部累计时间
} Signal;

void Signal_Init(Signal *sig ,float amplitude, float frequency, float offset, float phase );

/* 直接计算信号值（纯数学函数，给定自变量 x） */
float Signal_Step(float x);                                   // 阶跃函数
float Signal_Sin(float x);                                    // 正弦函数
float Signal_Linear(float x, float slope, float intercept);   // 一次函数
float Signal_Quadratic(float x, float a, float b, float c);   // 二次函数

/* 连续信号发生器（基于 dt 累加内部时间，返回当前输出值） */
float Signal_Step_Gen(Signal *sig, float dt);                 // 阶跃发生器
float Signal_Sin_Gen(Signal *sig, float dt);                  // 正弦发生器
float Signal_Ramp_Gen(Signal *sig, float dt);                 // 斜坡发生器（一次）
float Signal_Parabola_Gen(Signal *sig, float dt);             // 抛物线发生器（二次）

#endif
