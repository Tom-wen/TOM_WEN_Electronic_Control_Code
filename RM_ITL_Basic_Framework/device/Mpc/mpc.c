#include "mpc.h"
#include <math.h>
#include <string.h>

/* ================================================================
 *  共享工具函数（2x2矩阵/Cholesky/三角求解）
 * ================================================================ */
static void mat2_mul(const float A[4], const float B[4], float C[4])
{
    C[0] = A[0] * B[0] + A[1] * B[2];
    C[1] = A[0] * B[1] + A[1] * B[3];
    C[2] = A[2] * B[0] + A[3] * B[2];
    C[3] = A[2] * B[1] + A[3] * B[3];
}

static void mat2_vec(const float A[4], const float x[2], float y[2])
{
    y[0] = A[0] * x[0] + A[1] * x[1];
    y[1] = A[2] * x[0] + A[3] * x[1];
}

static void mat3_vec(const float A[9], const float x[3], float y[3])
{
    y[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2];
    y[1] = A[3] * x[0] + A[4] * x[1] + A[5] * x[2];
    y[2] = A[6] * x[0] + A[7] * x[1] + A[8] * x[2];
}

static int cholesky(const float *H, float *L, int n)
{
    for (int i = 0; i < n * n; i++) L[i] = 0.0f;
    for (int j = 0; j < n; j++)
    {
        float s = 0.0f;
        for (int k = 0; k < j; k++)
            s += L[j * n + k] * L[j * n + k];
        float diag = H[j * n + j] - s;
        if (diag <= 1e-12f) return 0;
        L[j * n + j] = sqrtf(diag);
        for (int i = j + 1; i < n; i++)
        {
            s = 0.0f;
            for (int k = 0; k < j; k++)
                s += L[i * n + k] * L[j * n + k];
            L[i * n + j] = (H[i * n + j] - s) / L[j * n + j];
        }
    }
    return 1;
}

static void forward_sub(const float *L, float *X, const float *B, int n)
{
    for (int i = 0; i < n; i++) X[i] = B[i];
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < i; j++)
            X[i] -= L[i * n + j] * X[j];
        X[i] /= L[i * n + i];
    }
}

static void backward_sub(const float *L, float *X, const float *B, int n)
{
    for (int i = 0; i < n; i++) X[i] = B[i];
    for (int i = n - 1; i >= 0; i--)
    {
        for (int j = i + 1; j < n; j++)
            X[i] -= L[j * n + i] * X[j];
        X[i] /= L[i * n + i];
    }
}

/* ================================================================
 *  PRE_MPC: 基础模型预测控制
 *  增广状态 [e, ∫e]^T, 一阶误差动力学
 *  有限时域N_p, 离线构建H/E矩阵, 在线每步QP求解（滚动优化）
 * ================================================================ */
#ifdef PRE_MPC

void MPC_Init(MPC *mpc, uint8_t N_p, float Q, float Q_i, float R,
              float A, float B, float maxI, float maxOut,
              float Kff, float dt, float dead_zone)
{
    mpc->N_p = N_p;
    mpc->Q   = Q;
    mpc->Q_i = Q_i;
    mpc->R   = R;
    mpc->A   = A;
    mpc->B   = B;
    mpc->maxIntegral = maxI;
    mpc->maxOutput   = maxOut;
    mpc->Kff   = Kff;
    mpc->dt    = dt;
    mpc->dead_zone = dead_zone;

    mpc->error     = 0.0f;
    mpc->lastError = 0.0f;
    mpc->integral  = 0.0f;
    mpc->Out       = 0.0f;
    mpc->last_reference = 0.0f;
    mpc->first_run = 1;
    mpc->solver_ready = 0;
    mpc->K_e = 0.0f;
    mpc->K_i = 0.0f;
    memset(mpc->H, 0, sizeof(mpc->H));
    memset(mpc->E, 0, sizeof(mpc->E));
    memset(mpc->L, 0, sizeof(mpc->L));

    if (N_p == 0 || N_p > MPC_MAX_NP || N_p > MPC_MAX_NC) return;

    int np = (int)N_p;

    /* 增广系统: x = [e, ie] */
    float A_aug[4] = { A, 0.0f, dt, 1.0f };
    float B_aug[2] = { B, 0.0f };
    float Q_aug[4] = { Q, 0.0f, 0.0f, Q_i };

    /* A_pow[i] = A_aug^i, i=0..np */
    static float A_pow[MPC_MAX_NP + 1][4];
    A_pow[0][0] = 1.0f; A_pow[0][1] = 0.0f;
    A_pow[0][2] = 0.0f; A_pow[0][3] = 1.0f;
    for (int i = 1; i <= np; i++)
        mat2_mul(A_pow[i - 1], A_aug, A_pow[i]);

    /* M: (np+1)*2 x 2, row-major: M[(b*2+s)*2 + k] = A_pow[b][s*2 + k] */
    static float M[(MPC_MAX_NP + 1) * 2 * 2];
    memset(M, 0, sizeof(M));
    for (int b = 0; b <= np; b++)
        for (int s = 0; s < 2; s++)
            for (int k = 0; k < 2; k++)
                M[(b * 2 + s) * 2 + k] = A_pow[b][s * 2 + k];

    /* C: (np+1)*2 x np, C[(b*2+s)*np + j] = (A_pow[b-1-j] * B_aug)[s] for b>j */
    static float C[(MPC_MAX_NP + 1) * 2 * MPC_MAX_NP];
    memset(C, 0, sizeof(C));
    for (int b = 1; b <= np; b++)
        for (int j = 0; j < b; j++)
        {
            float AB[2];
            mat2_vec(A_pow[b - 1 - j], B_aug, AB);
            C[(b * 2 + 0) * np + j] = AB[0];
            C[(b * 2 + 1) * np + j] = AB[1];
        }

    /* H = C^T * Q_bar * C + R_bar  (np x np) */
    for (int i = 0; i < np; i++)
        for (int j = 0; j < np; j++)
        {
            float sum = 0.0f;
            for (int b = 0; b <= np; b++)
            {
                float ci0 = C[(b * 2 + 0) * np + i];
                float ci1 = C[(b * 2 + 1) * np + i];
                float cj0 = C[(b * 2 + 0) * np + j];
                float cj1 = C[(b * 2 + 1) * np + j];
                float Qc0 = Q_aug[0] * cj0 + Q_aug[1] * cj1;
                float Qc1 = Q_aug[2] * cj0 + Q_aug[3] * cj1;
                sum += ci0 * Qc0 + ci1 * Qc1;
            }
            mpc->H[i * np + j] = sum + (i == j ? R : 0.0f);
        }

    /* E = C^T * Q_bar * M  (np x 2) */
    for (int i = 0; i < np; i++)
        for (int k = 0; k < 2; k++)
        {
            float sum = 0.0f;
            for (int b = 0; b <= np; b++)
            {
                float ci0 = C[(b * 2 + 0) * np + i];
                float ci1 = C[(b * 2 + 1) * np + i];
                float m0  = M[(b * 2 + 0) * 2 + k];
                float m1  = M[(b * 2 + 1) * 2 + k];
                float Qm0 = Q_aug[0] * m0 + Q_aug[1] * m1;
                float Qm1 = Q_aug[2] * m0 + Q_aug[3] * m1;
                sum += ci0 * Qm0 + ci1 * Qm1;
            }
            mpc->E[i * 2 + k] = sum;
        }

    /* Cholesky: H = L * L^T */
    mpc->solver_ready = cholesky(mpc->H, mpc->L, np) ? 1U : 0U;
}

void MPC_Calc(MPC *mpc, float reference, float feedback)
{
    if (mpc->N_p == 0 || mpc->N_p > MPC_MAX_NP || !mpc->solver_ready)
    {
        mpc->Out = 0.0f;
        mpc->first_run = 1;
        return;
    }

    mpc->error = reference - feedback;

    /* 死区 */
    float e_db = (fabsf(mpc->error) < mpc->dead_zone) ? 0.0f : mpc->error;

    /* 积分 + 抗饱和 */
    if (fabsf(mpc->Out) < mpc->maxOutput - 1e-3f)
    {
        mpc->integral += e_db * mpc->dt;
    }
    if (mpc->integral >  mpc->maxIntegral) mpc->integral =  mpc->maxIntegral;
    else if (mpc->integral < -mpc->maxIntegral) mpc->integral = -mpc->maxIntegral;

    /* 前馈 */
    float ff = mpc->first_run ? 0.0f : mpc->Kff * (reference - mpc->last_reference);

    /* ---- 滚动优化: 求解无约束QP min_U  0.5 U^T H U + (E*x)^T U ---- */
    int np = (int)mpc->N_p;
    float x[2] = { e_db, mpc->integral };

    /* f = E * x (线性项) */
    float f[MPC_MAX_NP];
    for (int i = 0; i < np; i++)
        f[i] = mpc->E[i * 2 + 0] * x[0] + mpc->E[i * 2 + 1] * x[1];

    /* 求解 L*L^T * U = -f  (Cholesky 三角回代) */
    float neg_f[MPC_MAX_NP];
    for (int i = 0; i < np; i++) neg_f[i] = -f[i];

    float Y[MPC_MAX_NP], U_opt[MPC_MAX_NP];
    forward_sub(mpc->L, Y, neg_f, np);
    backward_sub(mpc->L, U_opt, Y, np);

    /* 取第一个控制量 u = U*(0) */
    mpc->Out = U_opt[0] + ff;

    /* 输出钳位 (非约束优化,实际物理限幅) */
    if (mpc->Out >  mpc->maxOutput) mpc->Out =  mpc->maxOutput;
    else if (mpc->Out < -mpc->maxOutput) mpc->Out = -mpc->maxOutput;

    /* 刷新等效增益(用于监控) */
    if (fabsf(x[0]) > 1e-6f) mpc->K_e = U_opt[0] / x[0];
    else                      mpc->K_e = 0.0f;
    if (fabsf(x[1]) > 1e-6f) mpc->K_i = (U_opt[0] - mpc->K_e * x[0]) / x[1];
    else                      mpc->K_i = 0.0f;

    mpc->lastError      = mpc->error;
    mpc->last_reference = reference;
    mpc->first_run      = 0;
}

void MPC_CascadeCalc(MPC *mpc, float outerRef, float outerFdb, float innerFdb)
{
    MPC_Calc(&mpc[mpc_cascade_outer], outerRef, outerFdb);
    MPC_Calc(&mpc[mpc_cascade_inner], mpc[mpc_cascade_outer].Out, innerFdb);
}

void MPC_Clear(MPC *mpc)
{
    mpc->error     = 0.0f;
    mpc->lastError = 0.0f;
    mpc->integral  = 0.0f;
    mpc->Out       = 0.0f;
    mpc->last_reference = 0.0f;
    mpc->first_run = 1;
    mpc->K_e = 0.0f;
    mpc->K_i = 0.0f;
    /* solver_ready 不清零: Init 已构建 H/L, Clear 仅复位状态 */
}
#endif

/* ================================================================
 *  ALG_MPC: 拓展模型预测控制
 *  二阶离散状态空间 [e, de]^T
 *  离线构建H/E矩阵, 在线每步QP求解（滚动优化）
 *  配合积分器消除稳态误差
 * ================================================================ */
#ifdef ALG_MPC

void MPC_Init(MPC *mpc, uint8_t N_p, uint8_t N_c,
              float A11, float A12, float A21, float A22,
              float B1, float B2,
              float Q1, float Q2, float R, float K_F,
              float I_Out_Max, float Out_Max, float D_T,
              float Dead_Zone)
{
    mpc->N_p = N_p;
    mpc->N_c = N_c;
    mpc->A11 = A11; mpc->A12 = A12;
    mpc->A21 = A21; mpc->A22 = A22;
    mpc->B1 = B1; mpc->B2 = B2;
    mpc->Q1 = Q1; mpc->Q2 = Q2;
    mpc->R  = R;
    mpc->K_F = K_F;
    mpc->I_Out_Max = I_Out_Max;
    mpc->Out_Max   = Out_Max;
    mpc->D_T       = D_T;
    mpc->Dead_Zone = Dead_Zone;

    mpc->Pre_Error     = 0.0f;
    mpc->Pre_Error_Dot = 0.0f;
    mpc->Pre_Target    = 0.0f;
    mpc->Pre_Feedback  = 0.0f;
    mpc->Pre_Out       = 0.0f;
    mpc->Integral_Error = 0.0f;
    mpc->First_Run     = 1;
    mpc->Out           = 0.0f;
    mpc->Solver_Ready  = 0U;
    mpc->K[0] = 0.0f;
    mpc->K[1] = 0.0f;
    memset(mpc->H, 0, sizeof(mpc->H));
    memset(mpc->E, 0, sizeof(mpc->E));
    memset(mpc->L, 0, sizeof(mpc->L));

    if (N_p == 0 || N_p > MPC_MAX_NP) return;
    /* N_c 统一使用 N_p, 简化控制时域(PRE_MPC 风格) */
    int nc = (int)N_p;

    float A_mat[4] = { A11, A12, A21, A22 };
    float B_vec[2] = { B1, B2 };
    float Q_mat[4] = { Q1, 0.0f, 0.0f, Q2 };

    /* A_pow[i] = A_mat^i */
    static float A_pow[MPC_MAX_NP + 1][4];
    A_pow[0][0] = 1.0f; A_pow[0][1] = 0.0f;
    A_pow[0][2] = 0.0f; A_pow[0][3] = 1.0f;
    for (int i = 1; i <= nc; i++)
        mat2_mul(A_pow[i - 1], A_mat, A_pow[i]);

    /* M: (nc+1)*2 x 2 */
    static float M[(MPC_MAX_NP + 1) * 2 * 2];
    memset(M, 0, sizeof(M));
    for (int b = 0; b <= nc; b++)
        for (int s = 0; s < 2; s++)
            for (int k = 0; k < 2; k++)
                M[(b * 2 + s) * 2 + k] = A_pow[b][s * 2 + k];

    /* C: (nc+1)*2 x nc */
    static float C[(MPC_MAX_NP + 1) * 2 * MPC_MAX_NP];
    memset(C, 0, sizeof(C));
    for (int b = 1; b <= nc; b++)
        for (int j = 0; j < b; j++)
        {
            float AB[2];
            mat2_vec(A_pow[b - 1 - j], B_vec, AB);
            C[(b * 2 + 0) * nc + j] = AB[0];
            C[(b * 2 + 1) * nc + j] = AB[1];
        }

    /* H = C^T * Q_bar * C + R_bar */
    for (int i = 0; i < nc; i++)
        for (int j = 0; j < nc; j++)
        {
            float sum = 0.0f;
            for (int b = 0; b <= nc; b++)
            {
                float ci0 = C[(b * 2 + 0) * nc + i];
                float ci1 = C[(b * 2 + 1) * nc + i];
                float cj0 = C[(b * 2 + 0) * nc + j];
                float cj1 = C[(b * 2 + 1) * nc + j];
                float Qc0 = Q_mat[0] * cj0 + Q_mat[1] * cj1;
                float Qc1 = Q_mat[2] * cj0 + Q_mat[3] * cj1;
                sum += ci0 * Qc0 + ci1 * Qc1;
            }
            mpc->H[i * nc + j] = sum + (i == j ? R : 0.0f);
        }

    /* E = C^T * Q_bar * M */
    for (int i = 0; i < nc; i++)
        for (int k = 0; k < 2; k++)
        {
            float sum = 0.0f;
            for (int b = 0; b <= nc; b++)
            {
                float ci0 = C[(b * 2 + 0) * nc + i];
                float ci1 = C[(b * 2 + 1) * nc + i];
                float m0  = M[(b * 2 + 0) * 2 + k];
                float m1  = M[(b * 2 + 1) * 2 + k];
                float Qm0 = Q_mat[0] * m0 + Q_mat[1] * m1;
                float Qm1 = Q_mat[2] * m0 + Q_mat[3] * m1;
                sum += ci0 * Qm0 + ci1 * Qm1;
            }
            mpc->E[i * 2 + k] = sum;
        }

    mpc->Solver_Ready = cholesky(mpc->H, mpc->L, nc) ? 1U : 0U;
}

void MPC_CalcState(MPC *mpc, float reference, float feedback, float state_dot)
{
    if (mpc->N_p == 0 || mpc->N_p > MPC_MAX_NP || mpc->N_p > MPC_MAX_NC || !mpc->Solver_Ready)
    {
        mpc->Out = 0.0f;
        mpc->Pre_Out = 0.0f;
        mpc->First_Run = 1U;
        return;
    }

    float error_raw = reference - feedback;
    float error = error_raw;
    float abs_error = (error_raw > 0.0f) ? error_raw : -error_raw;
    float d_error = state_dot;

    /* 死区 */
    if (abs_error < mpc->Dead_Zone)
    {
        reference = feedback;
        error = 0.0f;
    }

    /* 积分 + 抗饱和 (fabsf 对称条件) */
    if (mpc->Out_Max == 0.0f || fabsf(mpc->Pre_Out) < mpc->Out_Max - 1e-3f)
    {
        mpc->Integral_Error += error * mpc->D_T;
    }
    if (mpc->I_Out_Max != 0.0f)
    {
        MPC_Math_Constrain(&mpc->Integral_Error, -mpc->I_Out_Max, mpc->I_Out_Max);
    }

    /* 前馈 */
    float f_out = mpc->First_Run ? 0.0f
                   : (reference - mpc->Pre_Target) * mpc->K_F;

    /* ---- 滚动优化 ---- */
    int nc = (int)mpc->N_p;
    float x[2] = { error, d_error };

    float f_vec[MPC_MAX_NP];
    for (int i = 0; i < nc; i++)
        f_vec[i] = mpc->E[i * 2 + 0] * x[0] + mpc->E[i * 2 + 1] * x[1];

    float neg_f[MPC_MAX_NP];
    for (int i = 0; i < nc; i++) neg_f[i] = -f_vec[i];

    float Y[MPC_MAX_NP], U_opt[MPC_MAX_NP];
    forward_sub(mpc->L, Y, neg_f, nc);
    backward_sub(mpc->L, U_opt, Y, nc);

    mpc->Out = U_opt[0] + mpc->Integral_Error + f_out;

    /* 输出限幅 */
    if (mpc->Out_Max != 0.0f)
    {
        MPC_Math_Constrain(&mpc->Out, -mpc->Out_Max, mpc->Out_Max);
    }

    /* 等效增益(监控) */
    mpc->K[0] = (fabsf(x[0]) > 1e-6f) ? (U_opt[0] / x[0]) : 0.0f;
    mpc->K[1] = (fabsf(x[1]) > 1e-6f) ? ((U_opt[0] - mpc->K[0] * x[0]) / x[1]) : 0.0f;

    mpc->Pre_Error     = error_raw;
    mpc->Pre_Error_Dot = d_error;
    mpc->Pre_Target    = reference;
    mpc->Pre_Feedback  = feedback;
    mpc->Pre_Out       = mpc->Out;
    mpc->First_Run     = 0;
}

void MPC_Calc(MPC *mpc, float reference, float feedback)
{
    /* derivative-on-measurement: 仅用反馈微分, 避免参考阶跃产生巨大脉冲 */
    float d_error = 0.0f;
    if (!mpc->First_Run)
    {
        d_error = -(feedback - mpc->Pre_Feedback) / mpc->D_T;
    }

    MPC_CalcState(mpc, reference, feedback, d_error);
}

void MPC_CascadeCalc(MPC *mpc, float outerRef, float outerFdb, float innerFdb)
{
    MPC_Calc(&mpc[mpc_cascade_outer], outerRef, outerFdb);
    MPC_Calc(&mpc[mpc_cascade_inner], mpc[mpc_cascade_outer].Out, innerFdb);
}

void MPC_Clear(MPC *mpc)
{
    mpc[mpc_single_loop].Out            = 0.0f;
    mpc[mpc_single_loop].Pre_Error      = 0.0f;
    mpc[mpc_single_loop].Pre_Error_Dot  = 0.0f;
    mpc[mpc_single_loop].Pre_Target     = 0.0f;
    mpc[mpc_single_loop].Pre_Feedback   = 0.0f;
    mpc[mpc_single_loop].Pre_Out        = 0.0f;
    mpc[mpc_single_loop].Integral_Error = 0.0f;
    mpc[mpc_single_loop].First_Run      = 1;

    mpc[mpc_cascade_inner].Out            = 0.0f;
    mpc[mpc_cascade_inner].Pre_Error      = 0.0f;
    mpc[mpc_cascade_inner].Pre_Error_Dot  = 0.0f;
    mpc[mpc_cascade_inner].Pre_Target     = 0.0f;
    mpc[mpc_cascade_inner].Pre_Feedback   = 0.0f;
    mpc[mpc_cascade_inner].Pre_Out        = 0.0f;
    mpc[mpc_cascade_inner].Integral_Error = 0.0f;
    mpc[mpc_cascade_inner].First_Run      = 1;

    mpc[mpc_cascade_outer].Out            = 0.0f;
    mpc[mpc_cascade_outer].Pre_Error      = 0.0f;
    mpc[mpc_cascade_outer].Pre_Error_Dot  = 0.0f;
    mpc[mpc_cascade_outer].Pre_Target     = 0.0f;
    mpc[mpc_cascade_outer].Pre_Feedback   = 0.0f;
    mpc[mpc_cascade_outer].Pre_Out        = 0.0f;
    mpc[mpc_cascade_outer].Integral_Error = 0.0f;
    mpc[mpc_cascade_outer].First_Run      = 1;
}

float MPC_Math_Abs(float x)
{
    return (x > 0.0f) ? x : -x;
}

void MPC_Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)      *x = Min;
    else if (*x > Max) *x = Max;
}

/* ================================================================
 *  Preview Tracking MPC
 *  Absolute-state tracking x = [pos, vel]^T with preview references
 * ================================================================ */
#define MPC_PREVIEW_G_INDEX(step, state, control) \
    ((((step) * MPC_STATE_DIM) + (state)) * MPC_MAX_NC + (control))
#define MPC_RAD_TO_DEG_F 57.29577951308232f

static float mpc_preview_wrap_angle_delta_deg(float delta_deg)
{
    if (delta_deg > 180.0f)
    {
        delta_deg -= 360.0f;
    }
    else if (delta_deg < -180.0f)
    {
        delta_deg += 360.0f;
    }

    return delta_deg;
}

static float mpc_preview_unwrap_angle_near_deg(float angle_deg, float near_deg)
{
    return near_deg + mpc_preview_wrap_angle_delta_deg(angle_deg - near_deg);
}

static void mpc_preview_simulate_free(const MPC_Preview *mpc, const float x0[2],
                                      float free_x[MPC_MAX_NP][2])
{
    const float A_mat[4] = { mpc->A11, mpc->A12, mpc->A21, mpc->A22 };
    float x[2] = { x0[0], x0[1] };

    for (int b = 0; b < (int)mpc->N_p; b++)
    {
        float x_next[2];
        mat2_vec(A_mat, x, x_next);
        free_x[b][0] = x_next[0];
        free_x[b][1] = x_next[1];
        x[0] = x_next[0];
        x[1] = x_next[1];
    }
}

void MPC_PreviewInit(MPC_Preview *mpc, uint8_t N_p, uint8_t N_c,
                     float A11, float A12, float A21, float A22,
                     float B1, float B2,
                     float Q1, float Q2,
                     float R_u, float R_delta,
                     float Out_Max, float Delta_Out_Max, float D_T)
{
    if (mpc == NULL) return;

    if (N_p == 0U || N_p > MPC_MAX_NP) N_p = MPC_MAX_NP;
    if (N_c == 0U) N_c = N_p;
    if (N_c > N_p) N_c = N_p;
    if (N_c > MPC_MAX_NC) N_c = MPC_MAX_NC;

    mpc->N_p = N_p;
    mpc->N_c = N_c;

    mpc->A11 = A11; mpc->A12 = A12;
    mpc->A21 = A21; mpc->A22 = A22;
    mpc->B1 = B1;   mpc->B2 = B2;

    mpc->Q1 = Q1;
    mpc->Q2 = Q2;
    mpc->R_u = R_u;
    mpc->R_delta = R_delta;

    mpc->Out_Max = Out_Max;
    mpc->Delta_Out_Max = Delta_Out_Max;
    mpc->D_T = D_T;
    mpc->Preview_Delay_S = 0.0f;

    mpc->Out = 0.0f;
    mpc->Last_Out = 0.0f;
    mpc->Solver_Ready = 0U;

    memset(mpc->H, 0, sizeof(mpc->H));
    memset(mpc->L, 0, sizeof(mpc->L));
    memset(mpc->G, 0, sizeof(mpc->G));

    if (N_p == 0U || N_c == 0U) return;

    const float A_mat[4] = { A11, A12, A21, A22 };
    const float B_vec[2] = { B1, B2 };
    const int np = (int)N_p;
    const int nc = (int)N_c;

    /* Build prediction matrix G column-by-column.
     * For j < N_c-1: basis input acts only at step j.
     * For j = N_c-1: last control is held constant for the remaining horizon.
     */
    for (int j = 0; j < nc; j++)
    {
        float x[2] = { 0.0f, 0.0f };
        for (int b = 0; b < np; b++)
        {
            float u_basis = 0.0f;
            if (j < nc - 1)
            {
                u_basis = (b == j) ? 1.0f : 0.0f;
            }
            else
            {
                u_basis = (b >= j) ? 1.0f : 0.0f;
            }

            float x_next[2];
            x_next[0] = A_mat[0] * x[0] + A_mat[1] * x[1] + B_vec[0] * u_basis;
            x_next[1] = A_mat[2] * x[0] + A_mat[3] * x[1] + B_vec[1] * u_basis;

            mpc->G[MPC_PREVIEW_G_INDEX(b, 0, j)] = x_next[0];
            mpc->G[MPC_PREVIEW_G_INDEX(b, 1, j)] = x_next[1];

            x[0] = x_next[0];
            x[1] = x_next[1];
        }
    }

    /* H = G^T Q_bar G + R_u I + R_delta D^T D */
    for (int i = 0; i < nc; i++)
        for (int j = 0; j < nc; j++)
        {
            float sum = 0.0f;
            for (int b = 0; b < np; b++)
            {
                const float gi0 = mpc->G[MPC_PREVIEW_G_INDEX(b, 0, i)];
                const float gi1 = mpc->G[MPC_PREVIEW_G_INDEX(b, 1, i)];
                const float gj0 = mpc->G[MPC_PREVIEW_G_INDEX(b, 0, j)];
                const float gj1 = mpc->G[MPC_PREVIEW_G_INDEX(b, 1, j)];
                sum += gi0 * Q1 * gj0 + gi1 * Q2 * gj1;
            }

            if (i == j) sum += R_u;

            if (R_delta > 0.0f)
            {
                float dtd = 0.0f;
                for (int k = 0; k < nc; k++)
                {
                    float dik = 0.0f;
                    float djk = 0.0f;

                    if (k == 0)
                    {
                        if (i == 0) dik = 1.0f;
                        if (j == 0) djk = 1.0f;
                    }
                    else
                    {
                        if (i == k)     dik = 1.0f;
                        else if (i == k - 1) dik = -1.0f;

                        if (j == k)     djk = 1.0f;
                        else if (j == k - 1) djk = -1.0f;
                    }

                    dtd += dik * djk;
                }
                sum += R_delta * dtd;
            }

            mpc->H[i * nc + j] = sum;
        }

    mpc->Solver_Ready = cholesky(mpc->H, mpc->L, nc) ? 1U : 0U;
}

void MPC_PreviewInitSimple(MPC_Preview *mpc, uint8_t N_p, uint8_t N_c,
                           float A11, float A12, float A21, float A22,
                           float B1, float B2,
                           float Q1, float Q2,
                           float R_u, float R_delta,
                           float Out_Max, float Delta_Out_Max,
                           float D_T, float Preview_Delay_S)
{
    MPC_PreviewInit(mpc, N_p, N_c,
                    A11, A12, A21, A22,
                    B1, B2,
                    Q1, Q2,
                    R_u, R_delta,
                    Out_Max, Delta_Out_Max,
                    D_T);

    if (mpc == NULL) return;

    mpc->Preview_Delay_S = (Preview_Delay_S > 0.0f) ? Preview_Delay_S : 0.0f;
}

void MPC_PreviewClear(MPC_Preview *mpc)
{
    if (mpc == NULL) return;
    mpc->Out = 0.0f;
    mpc->Last_Out = 0.0f;
}

void MPC_PreviewBuildConstAccel(const MPC_Preview *mpc,
                                float ref_pos, float ref_vel, float ref_acc,
                                float *ref_pos_seq, float *ref_vel_seq)
{
    if (mpc == NULL || ref_pos_seq == NULL || ref_vel_seq == NULL) return;

    for (int i = 0; i < (int)mpc->N_p; i++)
    {
        const float t = (float)(i + 1) * mpc->D_T;
        ref_pos_seq[i] = ref_pos + ref_vel * t + 0.5f * ref_acc * t * t;
        ref_vel_seq[i] = ref_vel + ref_acc * t;
    }
}

void MPC_PreviewCalc(MPC_Preview *mpc,
                     float feedback_pos, float feedback_vel,
                     const float *ref_pos_seq, const float *ref_vel_seq)
{
    if (mpc == NULL || ref_pos_seq == NULL || ref_vel_seq == NULL ||
        mpc->N_p == 0U || mpc->N_c == 0U || !mpc->Solver_Ready)
    {
        if (mpc != NULL)
        {
            mpc->Out = 0.0f;
            mpc->Last_Out = 0.0f;
        }
        return;
    }

    const int np = (int)mpc->N_p;
    const int nc = (int)mpc->N_c;

    float x0[2] = { feedback_pos, feedback_vel };
    float free_x[MPC_MAX_NP][2];
    float f_vec[MPC_MAX_NC];
    float neg_f[MPC_MAX_NC];
    float Y[MPC_MAX_NC];
    float U_opt[MPC_MAX_NC];

    mpc_preview_simulate_free(mpc, x0, free_x);

    for (int i = 0; i < nc; i++)
    {
        float sum = 0.0f;
        for (int b = 0; b < np; b++)
        {
            const float e_pos = free_x[b][0] - ref_pos_seq[b];
            const float e_vel = free_x[b][1] - ref_vel_seq[b];
            const float g0 = mpc->G[MPC_PREVIEW_G_INDEX(b, 0, i)];
            const float g1 = mpc->G[MPC_PREVIEW_G_INDEX(b, 1, i)];
            sum += g0 * mpc->Q1 * e_pos + g1 * mpc->Q2 * e_vel;
        }

        /* Delta-u term: only the first move depends on Last_Out directly. */
        if (i == 0) sum -= mpc->R_delta * mpc->Last_Out;

        f_vec[i] = sum;
        neg_f[i] = -sum;
    }

    forward_sub(mpc->L, Y, neg_f, nc);
    backward_sub(mpc->L, U_opt, Y, nc);

    mpc->Out = U_opt[0];

    if (mpc->Delta_Out_Max > 0.0f)
    {
        float delta_u = mpc->Out - mpc->Last_Out;
        MPC_Math_Constrain(&delta_u, -mpc->Delta_Out_Max, mpc->Delta_Out_Max);
        mpc->Out = mpc->Last_Out + delta_u;
    }

    if (mpc->Out_Max > 0.0f)
    {
        MPC_Math_Constrain(&mpc->Out, -mpc->Out_Max, mpc->Out_Max);
    }

    mpc->Last_Out = mpc->Out;
}

void MPC_PreviewCalcConstAccel(MPC_Preview *mpc,
                               float feedback_pos, float feedback_vel,
                               float ref_pos, float ref_vel, float ref_acc)
{
    float ref_pos_seq[MPC_MAX_NP];
    float ref_vel_seq[MPC_MAX_NP];

    MPC_PreviewBuildConstAccel(mpc, ref_pos, ref_vel, ref_acc, ref_pos_seq, ref_vel_seq);
    MPC_PreviewCalc(mpc, feedback_pos, feedback_vel, ref_pos_seq, ref_vel_seq);
}

void MPC_PreviewCalcSimple(MPC_Preview *mpc,
                           float target_acc_rad_s2,
                           float target_vel_rad_s,
                           float target_pos_deg,
                           float feedback_pos_deg,
                           float feedback_vel_rad_s)
{
    float ref_pos_seq[MPC_MAX_NP];
    float ref_vel_seq[MPC_MAX_NP];

    if (mpc == NULL || mpc->N_p == 0U || mpc->N_p > MPC_MAX_NP)
    {
        if (mpc != NULL)
        {
            mpc->Out = 0.0f;
            mpc->Last_Out = 0.0f;
        }
        return;
    }

    const float tau_s = (mpc->Preview_Delay_S > 0.0f) ? mpc->Preview_Delay_S : 0.0f;
    const float target_vel_deg_s = target_vel_rad_s * MPC_RAD_TO_DEG_F;
    const float target_acc_deg_s2 = target_acc_rad_s2 * MPC_RAD_TO_DEG_F;
    const float feedback_vel_deg_s = feedback_vel_rad_s * MPC_RAD_TO_DEG_F;

    float ref_pos_now_deg = target_pos_deg
                          + target_vel_deg_s * tau_s
                          + 0.5f * target_acc_deg_s2 * tau_s * tau_s;
    float ref_vel_now_deg_s = target_vel_deg_s + target_acc_deg_s2 * tau_s;

    ref_pos_now_deg = mpc_preview_unwrap_angle_near_deg(ref_pos_now_deg, feedback_pos_deg);

    MPC_PreviewBuildConstAccel(mpc,
                               ref_pos_now_deg,
                               ref_vel_now_deg_s,
                               target_acc_deg_s2,
                               ref_pos_seq,
                               ref_vel_seq);

    {
        float preview_near_deg = feedback_pos_deg;
        for (uint8_t i = 0U; i < mpc->N_p; i++)
        {
            ref_pos_seq[i] = mpc_preview_unwrap_angle_near_deg(ref_pos_seq[i], preview_near_deg);
            preview_near_deg = ref_pos_seq[i];
        }
    }

    MPC_PreviewCalc(mpc,
                    feedback_pos_deg,
                    feedback_vel_deg_s,
                    ref_pos_seq,
                    ref_vel_seq);
}

/* ================================================================
 *  3-state Preview Tracking MPC
 *  x = [pos, vel, dist]^T, dist is a constant additive disturbance
 * ================================================================ */
#define MPC_PREVIEW3_G_INDEX(step, state, control) \
    ((((step) * MPC_PREVIEW3_STATE_DIM) + (state)) * MPC_MAX_NC + (control))

static void mpc_preview3_simulate_free(const MPC_Preview3Dist *mpc, const float x0[3],
                                       float free_x[MPC_MAX_NP][3])
{
    const float A_mat[9] = {
        mpc->A11, mpc->A12, mpc->A13,
        mpc->A21, mpc->A22, mpc->A23,
        mpc->A31, mpc->A32, mpc->A33
    };
    float x[3] = { x0[0], x0[1], x0[2] };

    for (int b = 0; b < (int)mpc->N_p; b++)
    {
        float x_next[3];
        mat3_vec(A_mat, x, x_next);
        free_x[b][0] = x_next[0];
        free_x[b][1] = x_next[1];
        free_x[b][2] = x_next[2];
        x[0] = x_next[0];
        x[1] = x_next[1];
        x[2] = x_next[2];
    }
}

static void mpc_preview3_build_const_accel(const MPC_Preview3Dist *mpc,
                                           float ref_pos, float ref_vel, float ref_acc,
                                           float *ref_pos_seq, float *ref_vel_seq)
{
    if (mpc == NULL || ref_pos_seq == NULL || ref_vel_seq == NULL) return;

    for (int i = 0; i < (int)mpc->N_p; i++)
    {
        const float t = (float)(i + 1) * mpc->D_T;
        ref_pos_seq[i] = ref_pos + ref_vel * t + 0.5f * ref_acc * t * t;
        ref_vel_seq[i] = ref_vel + ref_acc * t;
    }
}

void MPC_Preview3DistInitSimple(MPC_Preview3Dist *mpc, uint8_t N_p, uint8_t N_c,
                                float A22, float B2,
                                float Q1, float Q2, float Q3,
                                float R_u, float R_delta,
                                float Out_Max, float Delta_Out_Max,
                                float D_T, float Preview_Delay_S,
                                float Dist_Est_Alpha)
{
    if (mpc == NULL) return;

    if (N_p == 0U || N_p > MPC_MAX_NP) N_p = MPC_MAX_NP;
    if (N_c == 0U) N_c = N_p;
    if (N_c > N_p) N_c = N_p;
    if (N_c > MPC_MAX_NC) N_c = MPC_MAX_NC;

    mpc->N_p = N_p;
    mpc->N_c = N_c;

    mpc->A11 = 1.0f; mpc->A12 = D_T;  mpc->A13 = 0.0f;
    mpc->A21 = 0.0f; mpc->A22 = A22;  mpc->A23 = 1.0f;
    mpc->A31 = 0.0f; mpc->A32 = 0.0f; mpc->A33 = 1.0f;
    mpc->B1 = 0.0f;  mpc->B2 = B2;    mpc->B3 = 0.0f;

    mpc->Q1 = Q1;
    mpc->Q2 = Q2;
    mpc->Q3 = Q3;
    mpc->R_u = R_u;
    if (mpc->R_u < 1e-6f) mpc->R_u = 1e-6f;       /* prevent Cholesky near-singular */
    mpc->R_delta = R_delta;

    mpc->Out_Max = Out_Max;
    mpc->Delta_Out_Max = Delta_Out_Max;
    mpc->D_T = D_T;
    mpc->Preview_Delay_S = (Preview_Delay_S > 0.0f) ? Preview_Delay_S : 0.0f;
    mpc->Dist_Est_Alpha = _constrain(Dist_Est_Alpha, 0.0f, 1.0f);

    mpc->Dist_Est = 0.0f;
    mpc->Prev_Feedback_Pos = 0.0f;
    mpc->Prev_Feedback_Vel = 0.0f;
    mpc->Dist_Est_Ready = 0U;
    mpc->Out = 0.0f;
    mpc->Last_Out = 0.0f;
    mpc->Solver_Ready = 0U;

    memset(mpc->H, 0, sizeof(mpc->H));
    memset(mpc->L, 0, sizeof(mpc->L));
    memset(mpc->G, 0, sizeof(mpc->G));

    if (N_p == 0U || N_c == 0U) return;

    const float A_mat[9] = {
        mpc->A11, mpc->A12, mpc->A13,
        mpc->A21, mpc->A22, mpc->A23,
        mpc->A31, mpc->A32, mpc->A33
    };
    const float B_vec[3] = { mpc->B1, mpc->B2, mpc->B3 };
    const int np = (int)N_p;
    const int nc = (int)N_c;

    for (int j = 0; j < nc; j++)
    {
        float x[3] = { 0.0f, 0.0f, 0.0f };
        for (int b = 0; b < np; b++)
        {
            float u_basis = 0.0f;
            if (j < nc - 1)
            {
                u_basis = (b == j) ? 1.0f : 0.0f;
            }
            else
            {
                u_basis = (b >= j) ? 1.0f : 0.0f;
            }

            float x_next[3];
            x_next[0] = A_mat[0] * x[0] + A_mat[1] * x[1] + A_mat[2] * x[2] + B_vec[0] * u_basis;
            x_next[1] = A_mat[3] * x[0] + A_mat[4] * x[1] + A_mat[5] * x[2] + B_vec[1] * u_basis;
            x_next[2] = A_mat[6] * x[0] + A_mat[7] * x[1] + A_mat[8] * x[2] + B_vec[2] * u_basis;

            mpc->G[MPC_PREVIEW3_G_INDEX(b, 0, j)] = x_next[0];
            mpc->G[MPC_PREVIEW3_G_INDEX(b, 1, j)] = x_next[1];
            mpc->G[MPC_PREVIEW3_G_INDEX(b, 2, j)] = x_next[2];

            x[0] = x_next[0];
            x[1] = x_next[1];
            x[2] = x_next[2];
        }
    }

    for (int i = 0; i < nc; i++)
        for (int j = 0; j < nc; j++)
        {
            float sum = 0.0f;
            for (int b = 0; b < np; b++)
            {
                const float gi0 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 0, i)];
                const float gi1 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 1, i)];
                const float gj0 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 0, j)];
                const float gj1 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 1, j)];
                sum += gi0 * Q1 * gj0 + gi1 * Q2 * gj1;
            }

            if (i == j) sum += R_u;

            if (R_delta > 0.0f)
            {
                float dtd = 0.0f;
                for (int k = 0; k < nc; k++)
                {
                    float dik = 0.0f;
                    float djk = 0.0f;

                    if (k == 0)
                    {
                        if (i == 0) dik = 1.0f;
                        if (j == 0) djk = 1.0f;
                    }
                    else
                    {
                        if (i == k) dik = 1.0f;
                        else if (i == k - 1) dik = -1.0f;

                        if (j == k) djk = 1.0f;
                        else if (j == k - 1) djk = -1.0f;
                    }

                    dtd += dik * djk;
                }
                sum += R_delta * dtd;
            }

            mpc->H[i * nc + j] = sum;
        }

    mpc->Solver_Ready = cholesky(mpc->H, mpc->L, nc) ? 1U : 0U;
}

void MPC_Preview3DistClear(MPC_Preview3Dist *mpc)
{
    if (mpc == NULL) return;
    mpc->Out = 0.0f;
    mpc->Last_Out = 0.0f;
    mpc->Dist_Est = 0.0f;
    mpc->Prev_Feedback_Pos = 0.0f;
    mpc->Prev_Feedback_Vel = 0.0f;
    mpc->Dist_Est_Ready = 0U;
}

void MPC_Preview3DistCalcSimple(MPC_Preview3Dist *mpc,
                                float target_acc_rad_s2,
                                float target_vel_rad_s,
                                float target_pos_deg,
                                float feedback_pos_deg,
                                float feedback_vel_rad_s)
{
    float ref_pos_seq[MPC_MAX_NP];
    float ref_vel_seq[MPC_MAX_NP];

    if (mpc == NULL || mpc->N_p == 0U || mpc->N_p > MPC_MAX_NP || !mpc->Solver_Ready)
    {
        if (mpc != NULL)
        {
            mpc->Out = 0.0f;
            mpc->Last_Out = 0.0f;
        }
        return;
    }

    const int np = (int)mpc->N_p;
    const int nc = (int)mpc->N_c;
    const float tau_s = (mpc->Preview_Delay_S > 0.0f) ? mpc->Preview_Delay_S : 0.0f;
    const float target_vel_deg_s = target_vel_rad_s * MPC_RAD_TO_DEG_F;
    const float target_acc_deg_s2 = target_acc_rad_s2 * MPC_RAD_TO_DEG_F;
    const float feedback_vel_deg_s = feedback_vel_rad_s * MPC_RAD_TO_DEG_F;

    if (!mpc->Dist_Est_Ready)
    {
        mpc->Prev_Feedback_Pos = feedback_pos_deg;
        mpc->Prev_Feedback_Vel = feedback_vel_deg_s;
        mpc->Dist_Est = 0.0f;
        mpc->Dist_Est_Ready = 1U;
    }
    else
    {
        const float pred_vel = mpc->A21 * mpc->Prev_Feedback_Pos +
                               mpc->A22 * mpc->Prev_Feedback_Vel +
                               mpc->A23 * mpc->Dist_Est +
                               mpc->B2 * mpc->Last_Out;
        const float innovation = feedback_vel_deg_s - pred_vel;
        mpc->Dist_Est += mpc->Dist_Est_Alpha * innovation;
        MPC_Math_Constrain(&mpc->Dist_Est, -50.0f, 50.0f);  /* anti-windup: deg/s */
        mpc->Prev_Feedback_Pos = feedback_pos_deg;
        mpc->Prev_Feedback_Vel = feedback_vel_deg_s;
    }

    float ref_pos_now_deg = target_pos_deg
                          + target_vel_deg_s * tau_s
                          + 0.5f * target_acc_deg_s2 * tau_s * tau_s;
    float ref_vel_now_deg_s = target_vel_deg_s + target_acc_deg_s2 * tau_s;

    ref_pos_now_deg = mpc_preview_unwrap_angle_near_deg(ref_pos_now_deg, feedback_pos_deg);

    mpc_preview3_build_const_accel(mpc,
                                   ref_pos_now_deg,
                                   ref_vel_now_deg_s,
                                   target_acc_deg_s2,
                                   ref_pos_seq,
                                   ref_vel_seq);

    {
        float preview_near_deg = feedback_pos_deg;
        for (uint8_t i = 0U; i < mpc->N_p; i++)
        {
            ref_pos_seq[i] = mpc_preview_unwrap_angle_near_deg(ref_pos_seq[i], preview_near_deg);
            preview_near_deg = ref_pos_seq[i];
        }
    }

    float x0[3] = { feedback_pos_deg, feedback_vel_deg_s, mpc->Dist_Est };
    float free_x[MPC_MAX_NP][3];
    float f_vec[MPC_MAX_NC];
    float neg_f[MPC_MAX_NC];
    float Y[MPC_MAX_NC];
    float U_opt[MPC_MAX_NC];

    mpc_preview3_simulate_free(mpc, x0, free_x);

    for (int i = 0; i < nc; i++)
    {
        float sum = 0.0f;
        for (int b = 0; b < np; b++)
        {
            const float e_pos = free_x[b][0] - ref_pos_seq[b];
            const float e_vel = free_x[b][1] - ref_vel_seq[b];
            const float g0 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 0, i)];
            const float g1 = mpc->G[MPC_PREVIEW3_G_INDEX(b, 1, i)];
            sum += g0 * mpc->Q1 * e_pos +
                   g1 * mpc->Q2 * e_vel;
        }

        if (i == 0) sum -= mpc->R_delta * mpc->Last_Out;

        f_vec[i] = sum;
        neg_f[i] = -sum;
    }

    forward_sub(mpc->L, Y, neg_f, nc);
    backward_sub(mpc->L, U_opt, Y, nc);

    mpc->Out = U_opt[0];

    if (mpc->Delta_Out_Max > 0.0f)
    {
        float delta_u = mpc->Out - mpc->Last_Out;
        MPC_Math_Constrain(&delta_u, -mpc->Delta_Out_Max, mpc->Delta_Out_Max);
        mpc->Out = mpc->Last_Out + delta_u;
    }

    if (mpc->Out_Max > 0.0f)
    {
        MPC_Math_Constrain(&mpc->Out, -mpc->Out_Max, mpc->Out_Max);
    }

    mpc->Last_Out = mpc->Out;
}
#endif
