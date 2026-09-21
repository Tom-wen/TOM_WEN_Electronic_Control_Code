#include "smc.h"

#ifndef SMC_OUTPUT_LPF_ALPHA
#define SMC_OUTPUT_LPF_ALPHA 0.85f
#endif

//一阶低通滤波，用于抑制SMC输出抖动
static float SMC_LowPassFilter(float input, float prev_output, float alpha)
{
    if (alpha <= 0.0f)
    {
        return input;
    }

    if (alpha >= 1.0f)
    {
        return prev_output;
    }

    return alpha * prev_output + (1.0f - alpha) * input;
}

//用于初始化SMC参数的函数
void SMC_Init(SMC *smc, float C, float K, float epsilon, float error_eps, float u_max, float J, float delta)
{
    smc->C = C;
    smc->K = K;
    smc->epsilon = epsilon;
    smc->error_eps = error_eps;
    smc->u_max = (u_max > 0.0f) ? u_max : 1.0f;
    smc->J = (J != 0.0f) ? J : 1.0f;
    smc->delta = delta;

    //初始化其他变量为0
    smc->ref = 0.0f;
    smc->refl = 0.0f;
    smc->dref = 0.0f;
    smc->ddref = 0.0f;
    smc->angle = 0.0f;
    smc->ang_vel = 0.0f;
    smc->error = 0.0f;
    smc->s = 0.0f;
    smc->u = 0.0f;
}

//设置SMC目标值
void SMC_SetRef(SMC *smc, float reference)
{
    smc->ref = reference;
}

//饱和函数 Sat(s)
float SMC_Sat(float s, float delta)
{
    if (delta <= 0.0f)
    {
        //无边界层，退化为符号函数
        if (s > 0.0f) return  1.0f;
        if (s < 0.0f) return -1.0f;
        return 0.0f;
    }
    if (s >  delta) return  1.0f;
    if (s < -delta) return -1.0f;
    return s / delta;
}

//SMC计算 angle_now为当前位置(°)，angle_vel为角速度(°/s)，dt为控制周期(s)
void SMC_Tick(SMC *smc, float angle_now, float angle_vel, float dt)
{
    float u_raw;
    float inv_dt;

    if (dt <= 0.0f) dt = 1e-3f;
    inv_dt = 1.0f / dt;

    //读取参数
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error = smc->angle - smc->ref;

    //前馈差分，除以dt统一单位为 °/s 和 °/s²
    smc->ddref = ((smc->ref - smc->refl) * inv_dt - smc->dref) * inv_dt;
    smc->dref  =  (smc->ref - smc->refl) * inv_dt;

    //误差下限处理
    if (fabs(smc->error) < smc->error_eps)
    {
        smc->u = 0.0f;
        smc->dref  = 0.0f;
        smc->ddref = 0.0f;
        //参数更新
        smc->refl = smc->ref;
        return;
    }

    //SMC滑模面
    smc->s = smc->C * smc->error + (smc->ang_vel - smc->dref);

    //控制律
    u_raw = smc->J * (smc->ddref
                     - smc->C * (smc->ang_vel - smc->dref)
                     - smc->epsilon * SMC_Sat(smc->s, smc->delta)
                     - smc->K * smc->s);

    //控制量限幅
    if (u_raw >  smc->u_max) u_raw =  smc->u_max;
    if (u_raw < -smc->u_max) u_raw = -smc->u_max;

    //对最终输出做一阶低通滤波，抑制抖振和尖峰
    smc->u = SMC_LowPassFilter(u_raw, smc->u, SMC_OUTPUT_LPF_ALPHA);

    //参数更新
    smc->refl = smc->ref;
}

//重置SMC状态
void SMC_Clear(SMC *smc)
{
    smc->ref = 0.0f;
    smc->refl = 0.0f;
    smc->dref = 0.0f;
    smc->ddref = 0.0f;
    smc->error = 0.0f;
    smc->s = 0.0f;
    smc->u = 0.0f;
}
