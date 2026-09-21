#include "rls.h"
#include <math.h>

  void RLS2D_Init(RLS2D *rls, float delta, float lambda)
  {
      rls->lambda     = lambda;
      rls->delta      = delta;
      rls->lastUpdate = 0;
      rls->updateCnt  = 0;
      rls->P[0][0] = delta;  rls->P[0][1] = 0.0f;
      rls->P[1][0] = 0.0f;   rls->P[1][1] = delta;
      rls->theta[0] = 0.0f;
      rls->theta[1] = 0.0f;
  }

  void RLS2D_SetParams(RLS2D *rls, float k1, float k2)
  {
      rls->theta[0] = k1;
      rls->theta[1] = k2;
  }

  void RLS2D_Reset(RLS2D *rls)
  {
      rls->P[0][0] = rls->delta;  rls->P[0][1] = 0.0f;
      rls->P[1][0] = 0.0f;        rls->P[1][1] = rls->delta;
      rls->theta[0] = 0.0f;
      rls->theta[1] = 0.0f;
  }

  void RLS2D_Update(RLS2D *rls, float x1, float x2, float y, float *k1_out, float *k2_out)
  {
      const float lambda = rls->lambda;

      // 1. Px = P * x (2x2 * 2x1)
      float Px0 = rls->P[0][0] * x1 + rls->P[0][1] * x2;
      float Px1 = rls->P[1][0] * x1 + rls->P[1][1] * x2;

      // 2. xTPx = x^T * P * x (标量)
      float xTPx = x1 * Px0 + x2 * Px1;

      // 3. 增益 K = Px / lambda / (1 + xTPx / lambda)
      float den = 1.0f + xTPx / lambda;
      float K0  = Px0 / lambda / den;
      float K1  = Px1 / lambda / den;

      // 4. 误差 = y - x^T * theta
      float err = y - (x1 * rls->theta[0] + x2 * rls->theta[1]);

      // 5. theta += K * err
      rls->theta[0] += K0 * err;
      rls->theta[1] += K1 * err;

      // 6. P = (P - K * x^T * P) / lambda
      //    K*x^T 是 2x2: [K0*x1, K0*x2; K1*x1, K1*x2]
      //    (K*x^T) * P = (K*x^T) * P
      float KxTP00 = (K0 * x1) * rls->P[0][0] + (K0 * x2) * rls->P[1][0];
      float KxTP01 = (K0 * x1) * rls->P[0][1] + (K0 * x2) * rls->P[1][1];
      float KxTP10 = (K1 * x1) * rls->P[0][0] + (K1 * x2) * rls->P[1][0];
      float KxTP11 = (K1 * x1) * rls->P[0][1] + (K1 * x2) * rls->P[1][1];

      rls->P[0][0] = (rls->P[0][0] - KxTP00) / lambda;
      rls->P[0][1] = (rls->P[0][1] - KxTP01) / lambda;
      rls->P[1][0] = (rls->P[1][0] - KxTP10) / lambda;
      rls->P[1][1] = (rls->P[1][1] - KxTP11) / lambda;

      // 输出约束（防止发散到负数）
      *k1_out = fmaxf(rls->theta[0], 1e-5f);
      *k2_out = fmaxf(rls->theta[1], 1e-5f);

      rls->updateCnt++;
      rls->lastUpdate = xTaskGetTickCount();
  }