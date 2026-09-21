
  #ifndef __RLS_H__
  #define __RLS_H__

  #include "FreeRTOS.h"
  #include "task.h"

  typedef struct {
      float P[2][2];      // 2x2 传递矩阵
      float theta[2];     // 参数向量 [k1, k2]
      float lambda;       // 遗忘因子 (0~1)
      float delta;        // P 初始对角值
      TickType_t lastUpdate;
      uint32_t updateCnt;
  } RLS2D;

  void RLS2D_Init(RLS2D *rls, float delta, float lambda);
  void RLS2D_SetParams(RLS2D *rls, float k1, float k2);
  void RLS2D_Reset(RLS2D *rls);
  void RLS2D_Update(RLS2D *rls, float x1, float x2, float y, float *k1_out, float *k2_out);

  #endif
