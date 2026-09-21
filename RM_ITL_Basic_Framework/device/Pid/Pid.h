#ifndef PID_H
#define PID_H

#include "main.h"
#include "Lowpass.h"
#include "config.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


#ifdef PRE_PID
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

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float Kff);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(PID *pid, float outerRef, float outerFdb, float innerFdb);
void PID_Clear(PID *pid);
#endif

#ifdef ALG_PID

/**
 * @brief 微分先行
 *
 */
typedef enum
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
} Enum_PID_D_First;

// 定义PID类型
typedef enum 
{
    single_loop = 0,    //单级PID
    cascade_inner = 1,  //串级PID内环
    cascade_outer = 2   //串级PID外环
} PID_Type;

/**
 * @brief PID结构体
 *
 */
typedef struct
{
    //初始化相关常量
    // PID计时器周期, s
    float D_T;
    //死区, Error在其绝对值内不输出
    float Dead_Zone;
    //微分先行
    Enum_PID_D_First D_First;

    //内部变量
    //之前的当前值
    float Pre_Now;
    //之前的目标值
    float Pre_Target;
    //之前的输出值
    float Pre_Out;
    //前向误差
    float Pre_Error;

    //读变量
    //输出值
    float Out;

    //写变量
    // PID的P
    float K_P;
    // PID的I
    float K_I;
    // PID的D
    float K_D;
    //前馈
    float K_F;

    //积分限幅, 0为不限制
    float I_Out_Max;
    //输出限幅, 0为不限制
    float Out_Max;

    //变速积分定速内段阈值, 0为不限制
    float I_Variable_Speed_A;
    //变速积分变速区间, 0为不限制
    float I_Variable_Speed_B;
    //积分分离阈值，需为正数, 0为不限制
    float I_Separate_Threshold;

    //读写变量
    //积分值
    float Integral_Error;
} PID;

/**
 * @brief 串级PID结构体
 *
 */
typedef struct
{
    PID inner;
    PID outer;
    float output;
}CascadePID;//串级pid

void PID_Init(PID *pid, float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T,
float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(PID *pid, float outerRef, float outerFdb, float innerFdb);
void PID_Clear(PID *pid);
float Math_Abs(float x);
void Math_Constrain(float *x, float Min, float Max);
#endif

#endif 
