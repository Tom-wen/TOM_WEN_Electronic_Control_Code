#include "Lowpass.h"

LowPassFilter  LPF_current,LPF_velocity, LPF_rc;

void LOWPass_Init()
{
    LPF_current.timestamp_prev= SysTick->VAL;
    LPF_current.Tf=0.1;

    LPF_velocity.timestamp_prev= SysTick->VAL;
    LPF_velocity.Tf=0.08;

    LPF_rc.timestamp_prev = SysTick->VAL;
    LPF_rc.Tf = 0.03;
}

float LowPassFilter_operator(LowPassFilter *Lfi, float x)
{
    uint32_t timestamp = SysTick->VAL;
    float Ts;

    if (timestamp > Lfi->timestamp_prev)
        Ts = (float)(timestamp - Lfi->timestamp_prev) / 60e6f;//单片机主频/8
    else
        Ts = (float)(0xFFFFFF - Lfi->timestamp_prev + timestamp) / 60e6f;

    if (Ts <= 0 || Ts > 0.1f) Ts = 0.001f;  // fallback 1ms

    float alpha = Lfi->Tf / (Lfi->Tf + Ts);
    float y = alpha * Lfi->y_prev + (1.0f - alpha) * x;

    Lfi->y_prev = y;
    Lfi->timestamp_prev = timestamp;

    return y;
}

