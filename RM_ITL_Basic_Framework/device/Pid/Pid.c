#include "Pid.h"

#ifdef PRE_PID
//用于初始化单环pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float Kff)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->Kff = Kff;
}

//单环PID计算(带低通滤波,前馈)
void PID_Calc(PID *pid, float reference, float feedback)
{
    // 对反馈值进行低通滤波
    // float filtered_feedback = feedback;
    // if(pid->feedbackFilter != NULL) {
    //     filtered_feedback = LowPassFilter_operator(pid->feedbackFilter, feedback);
    // }

    // 保存上一次误差
    float prevError = pid->lastError;
    
    // 计算当前误差
    pid->error = reference - feedback;

    // 死区处理（只归零误差，不直接return）
    if(fabs(pid->error) < 0.5) {
        pid->error = 0;
    }

    // 比例
    float pout = pid->error * pid->kp;

    // 微分（防止微分失效）
    float dout = (pid->error - prevError) * pid->kd;
    
    // 积分防风up
    if(fabs(pid->Out) < pid->maxOutput) { 
        pid->integral += pid->error * pid->ki;
        // 积分限幅
        if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
        else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    }

    //前馈
    float feedforward = pid->Kff * (reference - pid->last_reference); 

    // PID 总输出
    pid->Out = pout + dout + pid->integral + feedforward;

    // 输出限幅
    if(pid->Out > pid->maxOutput) pid->Out = pid->maxOutput;
    else if(pid->Out < -pid->maxOutput) pid->Out = -pid->maxOutput;

    // 更新lastError
    pid->lastError = pid->error;
    //更新last_last_reference
    pid->last_reference = reference;
}
//串级PID计算(带低通滤波,前馈)
void PID_CascadeCalc(PID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid[cascade_outer], outerRef, outerFdb);
    PID_Calc(&pid[cascade_inner], pid[cascade_outer].Out, innerFdb);
}

// 重置PID状态
void PID_Clear(PID *pid)
{
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->Out = 0;
    pid->last_reference = 0;
}

#endif

#ifdef ALG_PID

/**
 * @brief PID初始化
 * 注意: 1,必须给I最大限幅，不然就是没有限制I的输出！！！！！！！！
 *      2，KD范围别超过1，一定要给1以内，甚至可以不给！！！！！！！
 * @param pid PID结构体指针
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __K_F 前馈
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长度
 * @param __Dead_Zone 死区
 * @param __I_Variable_Speed_A 变速积分定速内段阈值
 * @param __I_Variable_Speed_B 变速积分变速区间
 * @param __I_Separate_Threshold 积分分离阈值
 * @param __D_First 微分先行
 */
void PID_Init(PID *pid, float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First)
{
    pid->K_P = __K_P;
    pid->K_I = __K_I;
    pid->K_D = __K_D;
    pid->K_F = __K_F;
    pid->I_Out_Max = __I_Out_Max;
    pid->Out_Max = __Out_Max;
    pid->D_T = __D_T;
    pid->Dead_Zone = __Dead_Zone;
    pid->I_Variable_Speed_A = __I_Variable_Speed_A;
    pid->I_Variable_Speed_B = __I_Variable_Speed_B;
    pid->I_Separate_Threshold = __I_Separate_Threshold;
    pid->D_First = __D_First;
    
    // 初始化其他变量为0
    pid->Pre_Now = 0.0f;
    pid->Pre_Target = 0.0f;
    pid->Pre_Out = 0.0f;
    pid->Pre_Error = 0.0f;
    pid->Out = 0.0f;
    pid->Integral_Error = 0.0f;
}

void PID_Calc(PID *pid, float reference, float feedback)
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    //误差
    float error;
    //绝对值误差
    float abs_error;
    //线性变速积分
    float speed_ratio;

    error = reference - feedback;
    abs_error = Math_Abs(error);

    //判断死区
    if (abs_error < pid->Dead_Zone)
    {
        reference = feedback;
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算p项
    p_out = pid->K_P * error;

    //计算i项
    if (pid->I_Variable_Speed_A == 0.0f && pid->I_Variable_Speed_B == 0.0f)
    {
        //非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        //变速积分
        if (abs_error <= pid->I_Variable_Speed_B)
        {
            speed_ratio = 1.0f;
        }
        else if (pid->I_Variable_Speed_B < abs_error && abs_error < pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
        {
            speed_ratio = (pid->I_Variable_Speed_A + pid->I_Variable_Speed_B - abs_error) / pid->I_Variable_Speed_A;
        }
        if (abs_error >= pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
        {
            speed_ratio = 0.0f;
        }
    }
      
    if (pid->I_Separate_Threshold == 0.0f)
    {
        //没有积分分离
        pid->Integral_Error += speed_ratio * pid->D_T * error;
        i_out = pid->K_I * pid->Integral_Error;
    }
    else
    {
        //积分分离使能
        if (abs_error < pid->I_Separate_Threshold)
        {
            pid->Integral_Error += speed_ratio * pid->D_T * error;
            i_out = pid->K_I * pid->Integral_Error;
        }
        else
        {
            pid->Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    //积分限幅
    if (pid->I_Out_Max != 0.0f && pid->K_I != 0)
    {
        Math_Constrain(&(pid->Integral_Error), -pid->I_Out_Max / pid->K_I, pid->I_Out_Max / pid->K_I);
    }

    //计算d项
    if (pid->D_First == PID_D_First_DISABLE)
    {
        //没有微分先行
        d_out = pid->K_D * (error - pid->Pre_Error) / pid->D_T;
    }
    else
    {
        //微分先行使能
        d_out = pid->K_D * (pid->Out - pid->Pre_Out) / pid->D_T;
        //d_out = -pid->K_D * (feedback - pid->Pre_Now) / pid->D_T;
    }

    //计算前馈
    f_out = (reference - pid->Pre_Target) * pid->K_F;

    //计算总共的输出
    pid->Out = p_out + i_out + d_out + f_out;
    
    //输出限幅
    if (pid->Out_Max != 0.0f)
    {
        Math_Constrain(&(pid->Out), -pid->Out_Max, pid->Out_Max);
    }

    //善后工作
    pid->Pre_Now = feedback;
    pid->Pre_Target = reference;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

//串级PID控制
void PID_CascadeCalc(PID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid[cascade_outer], outerRef, outerFdb);
    PID_Calc(&pid[cascade_inner], pid[cascade_outer].Out, innerFdb);
}

// 重置PID状态
void PID_Clear(PID *pid)
{
    pid[single_loop].Out = 0;
    pid[single_loop].Pre_Error = 0;
    pid[single_loop].Pre_Now = 0;
    pid[single_loop].Pre_Out = 0;
    pid[single_loop].Pre_Target = 0;
    pid[single_loop].Integral_Error = 0;

    pid[cascade_inner].Out = 0;
    pid[cascade_inner].Pre_Error = 0;
    pid[cascade_inner].Pre_Now = 0;
    pid[cascade_inner].Pre_Out = 0;
    pid[cascade_inner].Pre_Target = 0;
    pid[cascade_inner].Integral_Error = 0;
    
    pid[cascade_outer].Out = 0;
    pid[cascade_outer].Pre_Error = 0;
    pid[cascade_outer].Pre_Now = 0;
    pid[cascade_outer].Pre_Out = 0;
    pid[cascade_outer].Pre_Target = 0;
    pid[cascade_outer].Integral_Error = 0;
}


/**
 * @brief float类型求绝对值
 *
 * @param x 传入数据
 * @return float x的绝对值
 */
float Math_Abs(float x)
{
    return ((x > 0.0f) ? x : -x);
}
/**
 * @brief float类型限幅函数
 *
 * @param x 传入数据指针
 * @param Min 最小值
 * @param Max 最大值
 */
void Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

#endif
