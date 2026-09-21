#ifndef ADP_H
#define ADP_H

#include "main.h"
#include "Lowpass.h"

#ifndef _constrain
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

//条件编译
//基础ADP（Actor-Critic HDP）
#define PRE_ADP
//拓展ADP（带模型参考/归一化基函数）
//#define ALG_ADP

//状态维度：[error, integral_error, delta_error]
#define ADP_STATE_DIM   3
//Critic二次型基函数维度：phi = [e^2, e*ie, e*de, ie^2, ie*de, de^2]
#define ADP_CRITIC_DIM  6

#ifdef PRE_ADP
/**
 * @brief 基础自适应动态规划（ADP）结构体
 *         采用Actor-Critic架构，Critic使用二次型基函数逼近代价函数J(x),
 *         Actor使用线性策略u = Wa^T * x。通过TD误差在线更新权重。
 */
typedef struct
{
    //学习率与折扣因子
    float alpha_c;                  //Critic学习率
    float alpha_a;                  //Actor学习率
    float gamma;                    //折扣因子 (0~1)

    //代价函数权重
    float Q[ADP_STATE_DIM];         //状态代价权重 U = x^T Q x + R u^2
    float R;                        //控制代价权重

    //Actor权重（控制律 u = Wa^T * x）
    float Wa[ADP_STATE_DIM];
    //Critic权重（J(x) = Wc^T * phi(x)）
    float Wc[ADP_CRITIC_DIM];

    //状态
    float error, lastError;         //当前误差、上次误差
    float integral, maxIntegral;    //积分、积分限幅
    float delta_error;              //误差变化量
    float last_state[ADP_STATE_DIM];//上一时刻状态

    //TD学习暂存
    float last_J;                   //上一时刻代价估计
    float td_error;                 //当前TD误差

    //输出
    float Out, maxOutput;           //控制输出、输出限幅
    float u_basic;                  //基础(反馈)控制分量
    float u_adp;                    //ADP自适应补偿分量

    //前馈与滤波
    float Kff;                      //前馈系数
    float last_reference;           //上次参考值（用于前馈）
    LowPassFilter* feedbackFilter;  //反馈值低通滤波器指针

    //---- 算法修正新增字段 ----
    float dt;                       //采样周期, s
    float last_Out;                 //上一步实际输出（用于 TD 时间对齐）
    uint8_t first_run;              //首帧标志：1=首帧，跳过学习和 delta_error
    float Wc_max;                   //Critic 权重限幅（绝对值）
    float Wa_max;                   //Actor 权重限幅（绝对值）

    //---- v2 改进：de 一阶低通 + 状态归一化 + NLMS Critic ----
    float lambda_de;                //de 一阶低通系数 (0~1]，1=不滤波；默认 0.1
    float x_scale[ADP_STATE_DIM];   //状态归一化尺度 (>0)，默认 {1,1,1}=不归一化
    float de_filt;                  //滤波后 de 状态
    float td_error_clip;            //TD误差限幅（默认10.0）
} ADP;

//ADP类型
typedef enum
{
    adp_single_loop   = 0,          //单级ADP
    adp_cascade_inner = 1,          //串级ADP内环
    adp_cascade_outer = 2           //串级ADP外环
} ADP_Type;

void ADP_Init(ADP *adp, float alpha_c, float alpha_a, float gamma,
              float q1, float q2, float q3, float r,
              float maxI, float maxOut, float Kff,
              float dt, float Wc_max, float Wa_max);
void ADP_SetNorm(ADP *adp, float lambda_de, float sx0, float sx1, float sx2);
void ADP_Calc(ADP *adp, float reference, float feedback);
void ADP_CascadeCalc(ADP *adp, float outerRef, float outerFdb, float innerFdb);
void ADP_Clear(ADP *adp);
#endif

#ifdef ALG_ADP
/**
 * @brief 拓展ADP结构体
 *        支持自定义采样周期、死区、权重限幅、归一化基函数等。
 */
typedef struct
{
    //初始化常量
    float D_T;                      //采样周期, s
    float Dead_Zone;                //误差死区
    float Norm_Scale;               //状态归一化尺度, 0为不归一化

    //学习率与折扣
    float Alpha_C;                  //Critic学习率
    float Alpha_A;                  //Actor学习率
    float Gamma;                    //折扣因子

    //代价权重
    float Q[ADP_STATE_DIM];
    float R;

    //网络权重
    float Wa[ADP_STATE_DIM];
    float Wc[ADP_CRITIC_DIM];

    //权重限幅
    float Wa_Max;                   //Actor权重限幅, 0为不限制
    float Wc_Max;                   //Critic权重限幅, 0为不限制

    //内部变量
    float Pre_Error;                //上一拍"原始"误差（未经死区清零），用于 d_error
    float Pre_State[ADP_STATE_DIM];
    float Pre_J;
    float Pre_Out;
    float Pre_Target;
    float Integral_Error;
    uint8_t First_Run;              //首帧标志：1=首帧，跳过 d_error 和 TD 学习

    //读变量
    float Out;
    float TD_Error;

    //输出限幅
    float I_Out_Max;
    float Out_Max;

    //前馈
    float K_F;
} ADP;

/**
 * @brief 串级ADP结构体
 */
typedef struct
{
    ADP inner;
    ADP outer;
    float output;
} CascadeADP;

void ADP_Init(ADP *adp, float __Alpha_C, float __Alpha_A, float __Gamma,
              float __Q1, float __Q2, float __Q3, float __R, float __K_F,
              float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone,
              float __Norm_Scale, float __Wa_Max, float __Wc_Max);
void ADP_Calc(ADP *adp, float reference, float feedback);
void ADP_CascadeCalc(ADP *adp, float outerRef, float outerFdb, float innerFdb);
void ADP_Clear(ADP *adp);
float ADP_Math_Abs(float x);
void  ADP_Math_Constrain(float *x, float Min, float Max);
#endif

#endif
