#include "Pid.h"

//用于初始化单环pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float Kff)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->Kff = Kff;
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->Out = 0;
    pid->last_reference = 0;  // 重要！
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

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;

	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);

	return pid->out;
}
