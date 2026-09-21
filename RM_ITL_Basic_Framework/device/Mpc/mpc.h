#ifndef MPC_H
#define MPC_H

#include "main.h"
#include "Lowpass.h"

#ifndef _constrain
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

//条件编译
//基础MPC（一阶误差动力学+积分增广, 滚动QP优化）
// #define PRE_MPC
//拓展MPC（二阶动力学+Cholesky滚动优化+死区约束）
#define ALG_MPC

//状态维度：[error, integral_error] or [error, delta_error]
#define MPC_STATE_DIM   2
#define MPC_PREVIEW3_STATE_DIM 3

//MPC矩阵最大维度（N_p预测时域,N_c控制时域）
#define MPC_MAX_NP 20
#define MPC_MAX_NC 20

#ifdef PRE_MPC

/**
 * @brief 基础模型预测控制(MPC)结构体
 *        采用一阶误差动力学+积分增广模型，
 *        离线构建Hessian矩阵H和线性项矩阵E，
 *        在线每步求解无约束QP（滚动优化） u = U*(0) = argmin_U (0.5 U'H U + x'E U)
 *        控制律 = 最优反馈 + 前馈，不包含独立PD项。
 */
typedef struct
{
    //MPC预测参数
    uint8_t N_p;                    //预测时域长度
    float Q;                        //状态误差权重
    float Q_i;                      //积分误差权重
    float R;                        //控制代价权重

    //一阶系统模型: e(k+1) = A * e(k) + B * u(k)
    float A;                        //误差衰减系数 (0~1, 越小衰减越快)
    float B;                        //控制输入增益

    //预计算MPC增益（由Riccati方程离线解出）
    float K_e;                      //误差反馈增益
    float K_i;                      //积分反馈增益

    //MPC滚动优化矩阵（H=QP Hessian, E=线性项, L=Cholesky分解）
    float H[MPC_MAX_NP * MPC_MAX_NP];
    float E[MPC_MAX_NP * 2];
    float L[MPC_MAX_NP * MPC_MAX_NP];

    //状态变量
    float error, lastError;
    float integral, maxIntegral;

    //输出
    float Out, maxOutput;

    //前馈
    float Kff;                      //前馈系数
    float last_reference;           //上次参考值

    //采样时间
    float dt;

    //死区
    float dead_zone;

    //首帧标志
    uint8_t first_run;
    uint8_t solver_ready;
} MPC;

typedef enum
{
    mpc_single_loop   = 0,
    mpc_cascade_inner = 1,
    mpc_cascade_outer = 2
} MPC_Type;

void MPC_Init(MPC *mpc, uint8_t N_p, float Q, float Q_i, float R,
              float A, float B, float maxI, float maxOut,
              float Kff, float dt, float dead_zone);
void MPC_Calc(MPC *mpc, float reference, float feedback);
void MPC_CascadeCalc(MPC *mpc, float outerRef, float outerFdb, float innerFdb);
void MPC_Clear(MPC *mpc);
#endif

#ifdef ALG_MPC

/**
 * @brief 拓展MPC结构体
 *        支持二阶离散状态空间模型 [e, de]^T，
 *        离线构建预测矩阵H和E，在线每步通过Cholesky求解QP（滚动优化），
 *        配合积分器消除稳态误差。不包含独立PD项。
 */
typedef struct
{
    //MPC预测参数
    uint8_t N_p;                    //预测时域
    uint8_t N_c;                    //控制时域

    //二阶离散模型: [e(k+1); de(k+1)] = A * [e(k); de(k)] + B * u(k)
    float A11, A12, A21, A22;
    float B1, B2;

    //代价权重
    float Q1;                       //位置/误差权重
    float Q2;                       //速度/误差变化率权重
    float R;                        //控制权重

    //预计算增益（离线Riccati求解）
    float K[MPC_STATE_DIM];

    //MPC滚动优化矩阵（H=QP Hessian, E=线性项, L=Cholesky分解）
    float H[MPC_MAX_NC * MPC_MAX_NC];
    float E[MPC_MAX_NC * 2];
    float L[MPC_MAX_NC * MPC_MAX_NC];

    //死区
    float Dead_Zone;

    //输出限幅
    float Out_Max;
    float I_Out_Max;

    //内部状态
    float Pre_Error;
    float Pre_Error_Dot;
    float Pre_Target;
    float Pre_Feedback;             //上拍反馈值, 用于 derivative-on-measurement
    float Pre_Out;
    float Integral_Error;
    uint8_t First_Run;

    //前馈
    float K_F;

    //采样时间
    float D_T;

    //输出
    float Out;
    uint8_t Solver_Ready;
} MPC;

typedef struct
{
    MPC inner;
    MPC outer;
    float output;
} CascadeMPC;

typedef enum
{
    mpc_single_loop   = 0,
    mpc_cascade_inner = 1,
    mpc_cascade_outer = 2
} MPC_Type;

void MPC_Init(MPC *mpc, uint8_t N_p, uint8_t N_c,
              float A11, float A12, float A21, float A22,
              float B1, float B2,
              float Q1, float Q2, float R, float K_F,
              float I_Out_Max, float Out_Max, float D_T,
              float Dead_Zone);
void MPC_Calc(MPC *mpc, float reference, float feedback);
void MPC_CalcState(MPC *mpc, float reference, float feedback, float state_dot);
void MPC_CascadeCalc(MPC *mpc, float outerRef, float outerFdb, float innerFdb);
void MPC_Clear(MPC *mpc);
float MPC_Math_Abs(float x);
void  MPC_Math_Constrain(float *x, float Min, float Max);

/**
 * @brief Preview tracking MPC.
 *        Tracks absolute state x=[pos, vel]^T against preview reference
 *        sequences instead of regulating only the instantaneous error.
 * @note  For yaw-like circular states, caller should pass unwrapped preview
 *        references that stay continuous around the current feedback angle.
 */
typedef struct
{
    uint8_t N_p;
    uint8_t N_c;

    float A11, A12, A21, A22;
    float B1, B2;

    float Q1;
    float Q2;
    float R_u;
    float R_delta;

    float Out_Max;
    float Delta_Out_Max;
    float D_T;
    float Preview_Delay_S;

    float Out;
    float Last_Out;
    uint8_t Solver_Ready;

    float H[MPC_MAX_NC * MPC_MAX_NC];
    float L[MPC_MAX_NC * MPC_MAX_NC];
    float G[MPC_MAX_NP * MPC_STATE_DIM * MPC_MAX_NC];
} MPC_Preview;

void MPC_PreviewInit(MPC_Preview *mpc, uint8_t N_p, uint8_t N_c,
                     float A11, float A12, float A21, float A22,
                     float B1, float B2,
                     float Q1, float Q2,
                     float R_u, float R_delta,
                     float Out_Max, float Delta_Out_Max, float D_T);
void MPC_PreviewInitSimple(MPC_Preview *mpc, uint8_t N_p, uint8_t N_c,
                           float A11, float A12, float A21, float A22,
                           float B1, float B2,
                           float Q1, float Q2,
                           float R_u, float R_delta,
                           float Out_Max, float Delta_Out_Max,
                           float D_T, float Preview_Delay_S);
void MPC_PreviewClear(MPC_Preview *mpc);
void MPC_PreviewBuildConstAccel(const MPC_Preview *mpc,
                                float ref_pos, float ref_vel, float ref_acc,
                                float *ref_pos_seq, float *ref_vel_seq);
void MPC_PreviewCalc(MPC_Preview *mpc,
                     float feedback_pos, float feedback_vel,
                     const float *ref_pos_seq, const float *ref_vel_seq);
void MPC_PreviewCalcConstAccel(MPC_Preview *mpc,
                               float feedback_pos, float feedback_vel,
                               float ref_pos, float ref_vel, float ref_acc);
void MPC_PreviewCalcSimple(MPC_Preview *mpc,
                           float target_acc_rad_s2,
                           float target_vel_rad_s,
                           float target_pos_deg,
                           float feedback_pos_deg,
                           float feedback_vel_rad_s);

/**
 * @brief 3-state preview MPC with constant disturbance augmentation.
 *        x = [pos, vel, dist]^T
 *        pos(k+1) = pos(k) + dt * vel(k)
 *        vel(k+1) = A22 * vel(k) + dist(k) + B2 * u(k)
 *        dist(k+1) = dist(k)
 */
typedef struct
{
    uint8_t N_p;
    uint8_t N_c;

    float A11, A12, A13;
    float A21, A22, A23;
    float A31, A32, A33;
    float B1, B2, B3;

    float Q1;
    float Q2;
    float Q3;                       /* reserved – disturbance state is uncontrollable */
    float R_u;
    float R_delta;

    float Out_Max;
    float Delta_Out_Max;
    float D_T;
    float Preview_Delay_S;

    float Dist_Est_Alpha;           /* observer gain: 0=no update, 1=full innovation */
    float Dist_Est;
    float Prev_Feedback_Pos;
    float Prev_Feedback_Vel;
    uint8_t Dist_Est_Ready;

    float Out;
    float Last_Out;
    uint8_t Solver_Ready;

    float H[MPC_MAX_NC * MPC_MAX_NC];
    float L[MPC_MAX_NC * MPC_MAX_NC];
    float G[MPC_MAX_NP * MPC_PREVIEW3_STATE_DIM * MPC_MAX_NC];
} MPC_Preview3Dist;

void MPC_Preview3DistInitSimple(MPC_Preview3Dist *mpc, uint8_t N_p, uint8_t N_c,
                                float A22, float B2,
                                float Q1, float Q2, float Q3,
                                float R_u, float R_delta,
                                float Out_Max, float Delta_Out_Max,
                                float D_T, float Preview_Delay_S,
                                float Dist_Est_Alpha);
void MPC_Preview3DistClear(MPC_Preview3Dist *mpc);
void MPC_Preview3DistCalcSimple(MPC_Preview3Dist *mpc,
                                float target_acc_rad_s2,
                                float target_vel_rad_s,
                                float target_pos_deg,
                                float feedback_pos_deg,
                                float feedback_vel_rad_s);
#endif

#endif
