/**
 ******************************************************************************
 * @file    lqr_basic.c
 * @brief   LQR状态反馈控制器这些函数只适用于云台
 *
 *          默认LQR参数：MATLAB lqr()离线计算
 *          J_phi   = 13403.255/1e6 = 0.013403255 kg*m^2
 *          J_theta = 13024/1e6     = 0.013024    kg*m^2
 *          Q = diag([6000, 10, 100, 10])
 *          R = diag([1, 1])
 *
 *          K = [[77.4597,  3.4751,  0.0,     0.0    ],
 *               [0.0,      0.0,    10.0,    3.2032 ]]
 *
 *          闭环极点均在左半平面，系统稳定。
 ******************************************************************************
 */

#include "lqr_basic.h"

void LqrBasic_Init(LqrBasic *lqr, const float K[2][4], float phi_limit, float theta_limit)
{
    int i, j;

    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 4; j++)
        {
            lqr->K[i][j] = K[i][j];
        }
    }

    lqr->phi_limit   = (phi_limit   > 0.0f) ? phi_limit   : 1.0f;
    lqr->theta_limit = (theta_limit > 0.0f) ? theta_limit : 1.0f;

    LqrBasic_Clear(lqr);
}


//phi是yaw_error,yaw_target-yaw_fdb,phi_dot是yaw_error_dot,yaw_target_w-yaw_fdb_w,theta是pitch_error,pitch_target-pitch_fdb,theta_dot是pitch_error_w,pitch_target-pitch_fdb_w
void LqrBasic_Calc(LqrBasic *lqr, float phi, float phi_dot, float theta, float theta_dot)
{
    float u_phi;
    float u_theta;

    lqr->phi     = phi;
    lqr->phi_dot = phi_dot;
    lqr->theta   = theta;
    lqr->theta_dot = theta_dot;

    // u = -K * x
    u_phi   = -(lqr->K[0][0] * phi     + lqr->K[0][1] * phi_dot
              + lqr->K[0][2] * theta   + lqr->K[0][3] * theta_dot);

    u_theta = -(lqr->K[1][0] * phi     + lqr->K[1][1] * phi_dot
              + lqr->K[1][2] * theta   + lqr->K[1][3] * theta_dot);

    // 输出限幅
    if (u_phi > lqr->phi_limit)   u_phi   = lqr->phi_limit;
    if (u_phi < -lqr->phi_limit)  u_phi   = -lqr->phi_limit;
    if (u_theta > lqr->theta_limit)   u_theta = lqr->theta_limit;
    if (u_theta < -lqr->theta_limit)  u_theta = -lqr->theta_limit;

    lqr->tor_phi   = -u_phi;
    lqr->tor_theta = -u_theta;
}

void LqrBasic_SetK(LqrBasic *lqr, const float K[2][4])
{
    int i, j;

    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 4; j++)
        {
            lqr->K[i][j] = K[i][j];
        }
    }
}

void LqrBasic_Clear(LqrBasic *lqr)
{
    lqr->phi     = 0.0f;
    lqr->phi_dot = 0.0f;
    lqr->theta   = 0.0f;
    lqr->theta_dot = 0.0f;
    lqr->tor_phi   = 0.0f;
    lqr->tor_theta = 0.0f;
}
