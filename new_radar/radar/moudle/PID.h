#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"

typedef struct
{
  float kp;
  float ki;
  float kd;

  float i_max;
  float out_max;
  
  float ref; //??
  float fdb; //????
  float err[2];

  float p_out;
  float i_out;
  float d_out;
  float output;
}PID_struct_t;

// 模糊PID控制器结构体
typedef struct
{
  // 基本PID参数
  float kp;
  float ki;
  float kd;
  
  // 输出限幅
  float out_max;
  
  // 量化范围参数
  float e_max;      // 误差量化上限
  float e_min;      // 误差量化下限
  float ec_max;     // 误差变化率量化上限
  float ec_min;     // 误差变化率量化下限
  float kp_max;     // Kp增量上限
  float kp_min;     // Kp增量下限
  float ki_max;     // Ki增量上限
  float ki_min;     // Ki增量下限
  float kd_max;     // Kd增量上限
  float kd_min;     // Kd增量下限
  
  // 历史误差值
  float err[3];     // err[0]=当前误差, err[1]=上一次误差, err[2]=上上次误差
  
  // 输出
  float output;
  
  // 目标值和实际值
  float ref;
  float fdb;
}FuzzyPID_struct_t;

void PID_init(PID_struct_t *PID,float kp,float ki,float kd,float i_max,float out_max);
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb);
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb);
float PID_Calc_Ink(PID_struct_t *PID, float ref, float fdb);
//float PID_Calc_Follow(PID_struct_t *PID, float ref, float fdb);
int Limit_Min_Max(int value,int min,int max);

/**************************模糊PID控制器声明**********************************/

// 量化函数
float Quantization(float maximum, float minimum, float x);

// 反量化函数
float Inverse_quantization(float maximum, float minimum, float qvalues);

// 隶属度计算函数
void Get_grad_membership(float error, float error_c);

// 计算总隶属度函数
void GetSumGrad(void);

// 计算输出增量论域值函数
void GetOUT(void);

// 模糊PID控制器主函数（原版，保留兼容性）
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, 
                        float kp_max, float kp_min, float error, float error_c,
                        float ki_max, float ki_min, float kd_max, float kd_min,
                        float error_pre, float error_ppre);

// 简化版模糊PID初始化函数
void FuzzyPID_init(FuzzyPID_struct_t *fuzzy_pid,
                   float kp, float ki, float kd,
                   float out_max,
                   float e_max, float e_min,
                   float ec_max, float ec_min,
                   float kp_max, float kp_min,
                   float ki_max, float ki_min,
                   float kd_max, float kd_min);

// 简化版模糊PID计算函数（只需输入目标值和实际值）
float FuzzyPID_Calc(FuzzyPID_struct_t *fuzzy_pid, float ref, float fdb);

#ifdef __cplusplus
}
#endif

#endif
