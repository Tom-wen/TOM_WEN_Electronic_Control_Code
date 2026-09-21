#include "adp.h"
#include <math.h>

#ifdef PRE_ADP
/**
 * @brief 计算Critic二次型基函数 phi(x)
 *        phi = [e^2, e*ie, e*de, ie^2, ie*de, de^2]
 *        x 已是归一化坐标，调用方负责传入 x_n。
 */
static void ADP_Phi(const float x[ADP_STATE_DIM], float phi[ADP_CRITIC_DIM])
{
    phi[0] = x[0] * x[0];
    phi[1] = x[0] * x[1];
    phi[2] = x[0] * x[2];
    phi[3] = x[1] * x[1];
    phi[4] = x[1] * x[2];
    phi[5] = x[2] * x[2];
}

/**
 * @brief 把原始状态 x 按 x_scale 缩放到归一化坐标 x_n
 */
static void ADP_NormState(const ADP *adp,
                          const float x[ADP_STATE_DIM],
                          float       x_n[ADP_STATE_DIM])
{
    for (int i = 0; i < ADP_STATE_DIM; i++)
    {
        float s = adp->x_scale[i];
        x_n[i] = (s > 1e-9f) ? (x[i] / s) : x[i];
    }
}

/**
 * @brief 计算Critic输出 J(x) = Wc^T * phi(x)
 */
static float ADP_Critic(const float Wc[ADP_CRITIC_DIM], const float phi[ADP_CRITIC_DIM])
{
    float J = 0.0f;
    for (int i = 0; i < ADP_CRITIC_DIM; i++) J += Wc[i] * phi[i];
    return J;
}

/**
 * @brief 计算Actor输出 u(x) = Wa^T * x
 */
static float ADP_Actor(const float Wa[ADP_STATE_DIM], const float x[ADP_STATE_DIM])
{
    float u = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) u += Wa[i] * x[i];
    return u;
}

//初始化ADP参数
void ADP_Init(ADP *adp, float alpha_c, float alpha_a, float gamma,
              float q1, float q2, float q3, float r,
              float maxI, float maxOut, float Kff,
              float dt, float Wc_max, float Wa_max)
{
    adp->alpha_c = alpha_c;    
    adp->alpha_a = alpha_a;
    adp->gamma   = gamma;

    adp->Q[0] = q1; adp->Q[1] = q2; adp->Q[2] = q3;
    adp->R    = r;

    adp->maxIntegral = maxI;
    adp->maxOutput   = maxOut;
    adp->Kff         = Kff;

    //新增：采样周期、权重限幅、首帧标志、上步输出
    adp->dt        = (dt > 0.0f) ? dt : 0.002f;   //防 0 兜底
    adp->Wc_max    = (Wc_max > 0.0f) ? Wc_max : 100.0f;
    adp->Wa_max    = (Wa_max > 0.0f) ? Wa_max : 10.0f;
    adp->first_run = 1;
    adp->last_Out  = 0.0f;

    //v2：de 低通 + 状态归一化（默认 lambda=0.1，scale={1,1,1}=不归一化）
    adp->lambda_de   = 0.1f;
    adp->x_scale[0]  = 1.0f;
    adp->x_scale[1]  = 1.0f;
    adp->x_scale[2]  = 1.0f;
    adp->de_filt        = 0.0f;
    adp->td_error_clip  = 10.0f;

    //权重初值设为小正数，避免零初值导致无输出
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->Wa[i] = 0.0f;
    for (int i = 0; i < ADP_CRITIC_DIM; i++) adp->Wc[i] = 0.0f;
    //对角项(e^2, ie^2, de^2)初始化为1，形成正定初始代价
    adp->Wc[0] = 1.0f;
    adp->Wc[3] = 1.0f;
    adp->Wc[5] = 1.0f;
    //Actor默认做比例反馈
    adp->Wa[0] = 1.0f;

    adp->error = adp->lastError = 0.0f;
    adp->integral = 0.0f;
    adp->delta_error = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->last_state[i] = 0.0f;
    adp->last_J   = 0.0f;
    adp->td_error = 0.0f;
    adp->Out      = 0.0f;
    adp->u_basic  = 0.0f;
    adp->u_adp    = 0.0f;
    adp->last_reference = 0.0f;
}

/**
 * @brief 设置 de 低通系数与状态归一化尺度（在 ADP_Init 之后调用）
 *        - lambda_de: (0,1]，越小越平滑；1 表示不滤波。建议 0.05~0.2
 *        - sx0/sx1/sx2: e/ie/de 各自的尺度。让 (x_i/sx_i) 大致 O(1)。
 *          传入 0 或负值表示该维不归一化（保持原始量级）。
 */
void ADP_SetNorm(ADP *adp, float lambda_de, float sx0, float sx1, float sx2)
{
    if (lambda_de > 0.0f && lambda_de <= 1.0f) adp->lambda_de = lambda_de;
    adp->x_scale[0] = (sx0 > 0.0f) ? sx0 : 1.0f;
    adp->x_scale[1] = (sx1 > 0.0f) ? sx1 : 1.0f;
    adp->x_scale[2] = (sx2 > 0.0f) ? sx2 : 1.0f;
    adp->de_filt    = 0.0f;
}

//单环ADP计算（带死区、积分限幅、输出限幅、前馈与在线学习）
//v2 改动：
//  1) delta_error 经一阶低通，避免阶跃边沿尖峰；
//  2) Critic/Actor 在归一化坐标 x_n = x/x_scale 上工作，phi 各项量级一致；
//  3) Critic 用 NLMS 更新：ΔWc = α·δ·phi/(‖phi‖² + ε)，与 phi 模长解耦；
//  4) 死区只屏蔽 u_basic 的 P 项，学习路径用未死区化的 e（让 ie/de 学得动）。
void ADP_Calc(ADP *adp, float reference, float feedback)
{
    //保存上次"原始"误差（必须是死区前的值，否则 delta_error 会在退出死区瞬间产生假尖峰）
    float prevError = adp->lastError;

    //当前误差（先算原始值）
    adp->error = reference - feedback;
    float raw_error = adp->error;   //留一份给下一拍 lastError

    //误差变化：首帧置零防止冲击；其它帧除以 dt 得到时间导数后过一阶低通
    if (adp->first_run)
    {
        adp->delta_error = 0.0f;
        adp->de_filt     = 0.0f;
    }
    else
    {
        float de_raw    = (raw_error - prevError) / adp->dt;
        adp->de_filt    = (1.0f - adp->lambda_de) * adp->de_filt
                        + adp->lambda_de * de_raw;
        adp->delta_error = adp->de_filt;
    }

    //死区：只屏蔽 u_basic 的 P 项与积分累加，不影响微分与学习路径
    float e_db = (fabsf(adp->error) < 0.00005f) ? 0.0f : adp->error;

    //积分（带 dt）+ 基于上一拍输出的 anti-windup（用未死区化误差，保持学习信号）
    if (fabsf(adp->last_Out) < adp->maxOutput - 1e-3f)
    {
        adp->integral += adp->error * adp->dt;
    }
    if (adp->integral > adp->maxIntegral)       adp->integral = adp->maxIntegral;
    else if (adp->integral < -adp->maxIntegral) adp->integral = -adp->maxIntegral;

    //构造原始状态 x 与归一化状态 x_n
    float x[ADP_STATE_DIM];
    x[0] = adp->error;
    x[1] = adp->integral;
    x[2] = adp->delta_error;

    float x_n[ADP_STATE_DIM];
    ADP_NormState(adp, x, x_n);

    //Critic：J(x) = Wc^T phi(x_n)
    float phi[ADP_CRITIC_DIM];
    ADP_Phi(x_n, phi);
    float J = ADP_Critic(adp->Wc, phi);

    //Actor：u_adp_n = Wa^T x_n（归一化坐标下学习的策略）
    float u_adp = ADP_Actor(adp->Wa, x_n);

    //前馈（首帧 last_reference 尚未同步，强制置零避免上电尖峰）
    float feedforward = adp->first_run ? 0.0f
                                       : adp->Kff * (reference - adp->last_reference);

    //基础反馈分量（保持物理量级，等效 PD）
    float u_basic = adp->Q[0] * e_db + adp->Q[2] * adp->delta_error;

    //总输出 = 基础反馈 + ADP自适应补偿(归一化→物理) + 前馈
    adp->u_basic = u_basic;
    adp->u_adp   = u_adp ;
    adp->Out = u_basic + u_adp + feedforward;

    //输出限幅
    if (adp->Out > adp->maxOutput)       adp->Out = adp->maxOutput;
    else if (adp->Out < -adp->maxOutput) adp->Out = -adp->maxOutput;

    //---------- 在线学习（首帧跳过） ----------
    if (!adp->first_run)
    {
        //先归一化上一步状态，使 U 和 J 同坐标系，同时在归一化坐标下计算 phi
        float last_xn[ADP_STATE_DIM];
        ADP_NormState(adp, adp->last_state, last_xn);
        float last_phi[ADP_CRITIC_DIM];
        ADP_Phi(last_xn, last_phi);

        //效用函数 U(x_{k-1}, u_{k-1})：使用归一化坐标，与 J 尺度统一
        float last_out_n = adp->last_Out ;
        float U = adp->Q[0] * last_xn[0] * last_xn[0]
                + adp->Q[1] * last_xn[1] * last_xn[1]
                + adp->Q[2] * last_xn[2] * last_xn[2]
                + adp->R    * last_out_n  * last_out_n;

        //TD误差：delta = U_{k-1} + gamma * J(x_k) - J(x_{k-1})
        adp->td_error = U + adp->gamma * J - adp->last_J;
        if (adp->td_error >  adp->td_error_clip) adp->td_error =  adp->td_error_clip;
        if (adp->td_error < -adp->td_error_clip) adp->td_error = -adp->td_error_clip;

        //NLMS Critic 更新：ΔWc = α·δ·phi / (‖phi‖² + ε)
        float denom = 1e-3f;
        for (int i = 0; i < ADP_CRITIC_DIM; i++) denom += last_phi[i] * last_phi[i];
        float scale = adp->alpha_c * adp->td_error / denom;
        for (int i = 0; i < ADP_CRITIC_DIM; i++)
        {
            adp->Wc[i] += scale * last_phi[i];
            if (adp->Wc[i] >  adp->Wc_max) adp->Wc[i] =  adp->Wc_max;
            if (adp->Wc[i] < -adp->Wc_max) adp->Wc[i] = -adp->Wc_max;
        }

        //Actor权重更新（输出饱和时跳过，避免权重 windup）
        int saturated = (fabsf(adp->Out) >= adp->maxOutput - 1e-3f);
        if (!saturated)
        {
            //在归一化坐标下计算 dJ/dx_n
            float dJ_dxn[ADP_STATE_DIM];
            dJ_dxn[0] = 2.0f * adp->Wc[0] * x_n[0] + adp->Wc[1] * x_n[1] + adp->Wc[2] * x_n[2];
            dJ_dxn[1] = adp->Wc[1] * x_n[0] + 2.0f * adp->Wc[3] * x_n[1] + adp->Wc[4] * x_n[2];
            dJ_dxn[2] = adp->Wc[2] * x_n[0] + adp->Wc[4] * x_n[1] + 2.0f * adp->Wc[5] * x_n[2];

            float out_n = adp->Out;

            for (int i = 0; i < ADP_STATE_DIM; i++)
            {
                float grad = (2.0f * adp->R * out_n - adp->gamma * dJ_dxn[i]) * x_n[i];
                if (grad >  10.0f) grad =  10.0f;
                if (grad < -10.0f) grad = -10.0f;
                adp->Wa[i] -= adp->alpha_a * grad;
                if (adp->Wa[i] >  adp->Wa_max) adp->Wa[i] =  adp->Wa_max;
                if (adp->Wa[i] < -adp->Wa_max) adp->Wa[i] = -adp->Wa_max;
            }
        }
    }

    //---------- 状态迭代 ----------
    adp->last_J = J;
    adp->last_Out = adp->Out;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->last_state[i] = x[i];
    adp->lastError      = raw_error;   //存"死区前"原始误差，避免下一拍 delta_error 假尖峰
    adp->last_reference = reference;
    adp->first_run      = 0;
}

//串级ADP计算
void ADP_CascadeCalc(ADP *adp, float outerRef, float outerFdb, float innerFdb)
{
    ADP_Calc(&adp[adp_cascade_outer], outerRef, outerFdb);
    ADP_Calc(&adp[adp_cascade_inner], adp[adp_cascade_outer].Out, innerFdb);
}

//重置ADP状态（保留已学权重）
void ADP_Clear(ADP *adp)
{
    adp->error          = 0.0f;
    adp->lastError      = 0.0f;
    adp->integral       = 0.0f;
    adp->delta_error    = 0.0f;
    adp->de_filt        = 0.0f;
    adp->Out            = 0.0f;
    adp->u_basic        = 0.0f;
    adp->u_adp          = 0.0f;
    adp->last_J         = 0.0f;
    adp->td_error       = 0.0f;
    adp->last_reference = 0.0f;
    adp->last_Out       = 0.0f;
    adp->first_run      = 1;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->last_state[i] = 0.0f;
}
#endif

#ifdef ALG_ADP
/**
 * @brief 拓展ADP中的基函数（归一化后二次型）
 */
static void ADP_Phi_Ext(ADP *adp, const float x[ADP_STATE_DIM], float phi[ADP_CRITIC_DIM])
{
    float s[ADP_STATE_DIM];
    float k = (adp->Norm_Scale == 0.0f) ? 1.0f : (1.0f / adp->Norm_Scale);
    for (int i = 0; i < ADP_STATE_DIM; i++) s[i] = x[i] * k;

    phi[0] = s[0] * s[0];
    phi[1] = s[0] * s[1];
    phi[2] = s[0] * s[2];
    phi[3] = s[1] * s[1];
    phi[4] = s[1] * s[2];
    phi[5] = s[2] * s[2];
}

/**
 * @brief 拓展ADP初始化
 *        注意：1，必须给Actor/Critic权重限幅或给输出限幅，防止权重发散！
 *              2，Alpha_C / Alpha_A 需要根据系统尺度调小，通常 1e-4 ~ 1e-2。
 * @param adp          ADP结构体指针
 * @param __Alpha_C    Critic学习率
 * @param __Alpha_A    Actor学习率
 * @param __Gamma      折扣因子
 * @param __Q1/Q2/Q3   状态代价权重
 * @param __R          控制代价权重
 * @param __K_F        前馈系数
 * @param __I_Out_Max  积分限幅
 * @param __Out_Max    输出限幅
 * @param __D_T        采样周期
 * @param __Dead_Zone  误差死区
 * @param __Norm_Scale 状态归一化尺度
 * @param __Wa_Max     Actor权重限幅
 * @param __Wc_Max     Critic权重限幅
 */
void ADP_Init(ADP *adp, float __Alpha_C, float __Alpha_A, float __Gamma,
              float __Q1, float __Q2, float __Q3, float __R, float __K_F,
              float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone,
              float __Norm_Scale, float __Wa_Max, float __Wc_Max)
{
    adp->Alpha_C    = __Alpha_C;
    adp->Alpha_A    = __Alpha_A;
    adp->Gamma      = __Gamma;
    adp->Q[0]       = __Q1;
    adp->Q[1]       = __Q2;
    adp->Q[2]       = __Q3;
    adp->R          = __R;
    adp->K_F        = __K_F;
    adp->I_Out_Max  = __I_Out_Max;
    adp->Out_Max    = __Out_Max;
    adp->D_T        = __D_T;
    adp->Dead_Zone  = __Dead_Zone;
    adp->Norm_Scale = __Norm_Scale;
    adp->Wa_Max     = __Wa_Max;
    adp->Wc_Max     = __Wc_Max;

    for (int i = 0; i < ADP_STATE_DIM; i++) adp->Wa[i] = 0.0f;
    for (int i = 0; i < ADP_CRITIC_DIM; i++) adp->Wc[i] = 0.0f;
    adp->Wc[0] = 1.0f; adp->Wc[3] = 1.0f; adp->Wc[5] = 1.0f;
    adp->Wa[0] = 1.0f;

    adp->Pre_Error = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->Pre_State[i] = 0.0f;
    adp->Pre_J        = 0.0f;
    adp->Pre_Out      = 0.0f;
    adp->Pre_Target   = 0.0f;
    adp->Integral_Error = 0.0f;
    adp->Out          = 0.0f;
    adp->TD_Error     = 0.0f;
    adp->First_Run    = 1;
}

void ADP_Calc(ADP *adp, float reference, float feedback)
{
    //先算"原始"误差，再决定是否被死区屏蔽——Pre_Error/d_error 都必须用原始值
    float error_raw = reference - feedback;
    float error     = error_raw;
    float abs_error = ADP_Math_Abs(error_raw);

    //微分（首帧置零防止冲击；否则用原始 error 做后差分，避免死区污染）
    float d_error;
    if (adp->First_Run) d_error = 0.0f;
    else                d_error = (error_raw - adp->Pre_Error) / adp->D_T;

    //死区（只屏蔽 P/积分项，不影响已算好的微分）
    if (abs_error < adp->Dead_Zone)
    {
        reference = feedback;
        error     = 0.0f;
    }

    //积分（带 anti-windup）
    if (adp->Out_Max == 0.0f || ADP_Math_Abs(adp->Pre_Out) < adp->Out_Max - 1e-3f)
    {
        adp->Integral_Error += error * adp->D_T;
    }
    if (adp->I_Out_Max != 0.0f)
    {
        ADP_Math_Constrain(&adp->Integral_Error, -adp->I_Out_Max, adp->I_Out_Max);
    }

    //状态向量
    float x[ADP_STATE_DIM];
    x[0] = error;
    x[1] = adp->Integral_Error;
    x[2] = d_error;

    //Critic：J(x) = Wc^T phi(x)
    float phi[ADP_CRITIC_DIM];
    ADP_Phi_Ext(adp, x, phi);
    float J = 0.0f;
    for (int i = 0; i < ADP_CRITIC_DIM; i++) J += adp->Wc[i] * phi[i];

    //Actor：u = Wa^T x
    float u_adp = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) u_adp += adp->Wa[i] * x[i];

    //前馈（首帧 Pre_Target 未同步，置零避免上电尖峰）
    float f_out = adp->First_Run ? 0.0f : (reference - adp->Pre_Target) * adp->K_F;

    //总输出
    adp->Out = u_adp + f_out;
    if (adp->Out_Max != 0.0f)
    {
        ADP_Math_Constrain(&adp->Out, -adp->Out_Max, adp->Out_Max);
    }

    //---------- 在线学习（首帧跳过） ----------
    if (!adp->First_Run)
    {
        //效用函数 U(x_{k-1}, u_{k-1})：必须用"上一拍"的 state 和 output 才满足 Bellman
        float U = adp->Q[0] * adp->Pre_State[0] * adp->Pre_State[0]
                + adp->Q[1] * adp->Pre_State[1] * adp->Pre_State[1]
                + adp->Q[2] * adp->Pre_State[2] * adp->Pre_State[2]
                + adp->R    * adp->Pre_Out      * adp->Pre_Out;

        //TD误差：delta = U_{k-1} + gamma * J(x_k) - J(x_{k-1})
        adp->TD_Error = U + adp->Gamma * J - adp->Pre_J;

        //Critic更新：Wc += alpha_c * delta * phi(x_{k-1})
        //半梯度推导：d(delta)/d(Wc) = -last_phi，代入 Wc -= alpha*delta*d(delta)/dWc 得 +=
        float last_phi[ADP_CRITIC_DIM];
        ADP_Phi_Ext(adp, adp->Pre_State, last_phi);
        for (int i = 0; i < ADP_CRITIC_DIM; i++)
        {
            adp->Wc[i] += adp->Alpha_C * adp->TD_Error * last_phi[i];
            if (adp->Wc_Max != 0.0f)
            {
                ADP_Math_Constrain(&adp->Wc[i], -adp->Wc_Max, adp->Wc_Max);
            }
        }

        //Actor更新（输出饱和时跳过，避免权重 windup）
        int saturated = (adp->Out_Max != 0.0f) &&
                        (ADP_Math_Abs(adp->Out) >= adp->Out_Max - 1e-3f);
        if (!saturated)
        {
            float dJ_dx[ADP_STATE_DIM];
            dJ_dx[0] = 2.0f * adp->Wc[0] * x[0] + adp->Wc[1] * x[1] + adp->Wc[2] * x[2];
            dJ_dx[1] = adp->Wc[1] * x[0] + 2.0f * adp->Wc[3] * x[1] + adp->Wc[4] * x[2];
            dJ_dx[2] = adp->Wc[2] * x[0] + adp->Wc[4] * x[1] + 2.0f * adp->Wc[5] * x[2];

            for (int i = 0; i < ADP_STATE_DIM; i++)
            {
                float grad = (2.0f * adp->R * adp->Out + adp->Gamma * dJ_dx[i]) * x[i];
                adp->Wa[i] -= adp->Alpha_A * grad;
                if (adp->Wa_Max != 0.0f)
                {
                    ADP_Math_Constrain(&adp->Wa[i], -adp->Wa_Max, adp->Wa_Max);
                }
            }
        }
    }

    //善后（Pre_Error 存"原始"误差，避免下一拍 d_error 受死区污染）
    adp->Pre_Error  = error_raw;
    adp->Pre_J      = J;
    adp->Pre_Out    = adp->Out;
    adp->Pre_Target = reference;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp->Pre_State[i] = x[i];
    adp->First_Run  = 0;
}

//串级ADP计算
void ADP_CascadeCalc(ADP *adp, float outerRef, float outerFdb, float innerFdb)
{
    ADP_Calc(&adp[adp_cascade_outer], outerRef, outerFdb);
    ADP_Calc(&adp[adp_cascade_inner], adp[adp_cascade_outer].Out, innerFdb);
}

//重置ADP内部状态
void ADP_Clear(ADP *adp)
{
    adp[adp_single_loop].Out             = 0.0f;
    adp[adp_single_loop].Pre_Error       = 0.0f;
    adp[adp_single_loop].Pre_J           = 0.0f;
    adp[adp_single_loop].Pre_Out         = 0.0f;
    adp[adp_single_loop].Pre_Target      = 0.0f;
    adp[adp_single_loop].Integral_Error  = 0.0f;
    adp[adp_single_loop].TD_Error        = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp[adp_single_loop].Pre_State[i] = 0.0f;
    adp[adp_single_loop].First_Run = 1;

    adp[adp_cascade_inner].Out            = 0.0f;
    adp[adp_cascade_inner].Pre_Error      = 0.0f;
    adp[adp_cascade_inner].Pre_J          = 0.0f;
    adp[adp_cascade_inner].Pre_Out        = 0.0f;
    adp[adp_cascade_inner].Pre_Target     = 0.0f;
    adp[adp_cascade_inner].Integral_Error = 0.0f;
    adp[adp_cascade_inner].TD_Error       = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp[adp_cascade_inner].Pre_State[i] = 0.0f;
    adp[adp_cascade_inner].First_Run = 1;

    adp[adp_cascade_outer].Out            = 0.0f;
    adp[adp_cascade_outer].Pre_Error      = 0.0f;
    adp[adp_cascade_outer].Pre_J          = 0.0f;
    adp[adp_cascade_outer].Pre_Out        = 0.0f;
    adp[adp_cascade_outer].Pre_Target     = 0.0f;
    adp[adp_cascade_outer].Integral_Error = 0.0f;
    adp[adp_cascade_outer].TD_Error       = 0.0f;
    for (int i = 0; i < ADP_STATE_DIM; i++) adp[adp_cascade_outer].Pre_State[i] = 0.0f;
    adp[adp_cascade_outer].First_Run = 1;
}

/**
 * @brief float类型求绝对值
 */
float ADP_Math_Abs(float x)
{
    return (x > 0.0f) ? x : -x;
}

/**
 * @brief float类型限幅函数
 */
void ADP_Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)      *x = Min;
    else if (*x > Max) *x = Max;
}
#endif
