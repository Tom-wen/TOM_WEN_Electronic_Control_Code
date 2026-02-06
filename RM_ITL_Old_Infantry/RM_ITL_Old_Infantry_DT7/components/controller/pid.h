#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx.h"

typedef struct
{
  float kp;
  float ki;
  float kd;

  float i_max;
  float out_max;
  
  float ref; //�趨
  float fdb; //����
  float err[2];

  float p_out;
  float i_out;
  float d_out;
  float output;
}PID_struct_t;

void PID_Init(PID_struct_t *PID,float kp,float ki,float kd,float i_max,float out_max);
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb);//PID���㺯����Ŀ�꣬ʵ��)
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb);//PID���㺯����Ŀ�꣬ʵ�ʣ�
float PID_Calc_Ink(PID_struct_t *PID, float ref, float fdb);//PID���㺯����Ŀ�꣬ʵ�ʣ�
//float PID_Calc_Follow(PID_struct_t *PID, float ref, float fdb);//PID���㺯����Ŀ�꣬ʵ�ʣ�
int Limit_Min_Max(int value,int min,int max);

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          pid结构数据初始化
  * 
  * @param[out]     pid: PID结构数据指针
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
 */
extern void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float max_out, float max_iout);

/**
 * @brief          pid计算
 * 
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);

extern float PID_calc_chassis(pid_type_def *pid, float ref, float set);


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

extern short PID_deadline_limit(short *input, float deadline);

#ifdef __cplusplus
}
#endif

#endif


