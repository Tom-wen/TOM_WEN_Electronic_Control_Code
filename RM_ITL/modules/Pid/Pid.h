#ifndef PID_H
#define PID_H

#include "main.h"
#include "Lowpass.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//条件编译

//首先定义PID结构体用于存放一个PID的数据
typedef struct
{
   	float kp, ki, kd;               //三个系数
    float error, lastError;         //误差、上次误差
    float integral, maxIntegral;    //积分、积分限幅
    float Out, maxOutput;        //输出、输出限幅
    float Kff;                      //前馈系数
    float last_reference;  // 上一次外环目标值（用于前馈计算）
    LowPassFilter* feedbackFilter;  // 指向反馈值滤波器的指针
}PID;

// 定义PID类型
typedef enum 
{
    single_loop = 0,    //单级PID
    cascade_inner = 1,  //串级PID内环
    cascade_outer = 2   //串级PID外环
} PID_Type;

typedef struct
{
    //PID 
    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  
    float error[3]; 

} pid_type_def;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float Kff);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(PID *pid, float outerRef, float outerFdb, float innerFdb);
void PID_Clear(PID *pid);
float PID_calc(pid_type_def *pid, float ref, float set);

#endif
