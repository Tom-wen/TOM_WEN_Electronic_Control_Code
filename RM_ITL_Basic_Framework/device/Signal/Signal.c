#include "Signal.h"
#include <math.h>
#include <string.h>

#define SIGNAL_PI 3.141592653589793f

void Signal_Init(Signal *sig ,float amplitude, float frequency, float offset, float phase )
{
    sig->amplitude = amplitude;
    sig->frequency = frequency;
    sig->offset = offset;
    sig->phase = phase;
    sig->time = 0.0f;
}

/* 单位阶跃函数 u(x)：x >= 0 时输出 1，否则输出 0 */
float Signal_Step(float x)
{
    return (x >= 0.0f) ? 1.0f : 0.0f;
}

/* 正弦函数 sin(x) */
float Signal_Sin(float x)
{
    return sinf(x);
}

/* 一次函数 slope * x + intercept */
float Signal_Linear(float x, float slope, float intercept)
{
    return slope * x + intercept;
}

/* 二次函数 a * x² + b * x + c */
float Signal_Quadratic(float x, float a, float b, float c)
{
    return a * x * x + b * x + c;
}

/* 阶跃信号发生器（含幅值/偏置） */
float Signal_Step_Gen(Signal *sig, float dt)
{
    sig->time += dt;
    return (sig->time >= 0.0f) ? (sig->amplitude + sig->offset) : sig->offset;
}

/* 正弦信号发生器：amplitude * sin(2π * freq * t + phase) + offset */
float Signal_Sin_Gen(Signal *sig, float dt)
{
    sig->time += dt;
    return sig->amplitude * sinf(2.0f * SIGNAL_PI * sig->frequency * sig->time + sig->phase) + sig->offset;
}

/* 斜坡信号发生器（一次函数随时间变化）：amplitude * t + offset */
float Signal_Ramp_Gen(Signal *sig, float dt)
{
    sig->time += dt;
    return sig->amplitude * sig->time + sig->offset;
}

/* 抛物线信号发生器（二次函数随时间变化）：amplitude * t² + offset */
float Signal_Parabola_Gen(Signal *sig, float dt)
{
    sig->time += dt;
    return sig->amplitude * sig->time * sig->time + sig->offset;
}
