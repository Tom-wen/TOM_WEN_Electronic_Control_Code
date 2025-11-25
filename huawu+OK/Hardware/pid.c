#include "pid.h"
#include "gray_line.h"

// PID参数 - 需要根据实际调试调整
float line_kp = 80.0f;    // 巡线比例系数
float line_ki = 0.1f;     // 巡线积分系数  
float line_kd = 15.0f;    // 巡线微分系数

float speed_kp = 1.0f;    // 速度比例系数
float speed_ki = 0.05f;   // 速度积分系数
float speed_kd = 0.1f;    // 速度微分系数

pid_t line_pid;
pid_t speed_pid;

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->target = 0;
    pid->now = 0;
    pid->out = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    for(int i = 0; i < 3; i++) {
        pid->error[i] = 0;
    }
}

void pid_reset(pid_t *pid)
{
    pid->out = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    for(int i = 0; i < 3; i++) {
        pid->error[i] = 0;
    }
}

void pid_cal(pid_t *pid)
{
    // 计算当前偏差
    pid->error[0] = pid->target - pid->now;

    if(pid->pid_mode == POSITION_PID)  // 位置式PID
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }
    else if(pid->pid_mode == DELTA_PID)  // 增量式PID
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    }

    // 记录前两次偏差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

// 巡线PID控制
float line_pid_control(float current_position)
{
    line_pid.now = current_position;
    line_pid.target = 0.0f;  // 目标：保持在中心位置
    
    pid_cal(&line_pid);
    
    // 输出限幅
    if(line_pid.out > 500.0f) line_pid.out = 500.0f;
    if(line_pid.out < -500.0f) line_pid.out = -500.0f;
    
    return line_pid.out;
}

// 速度PID控制
float speed_pid_control(float current_speed, float target_speed)
{
    speed_pid.now = current_speed;
    speed_pid.target = target_speed;
    
    pid_cal(&speed_pid);
    
    // 输出限幅
    if(speed_pid.out > 800.0f) speed_pid.out = 800.0f;
    if(speed_pid.out < -800.0f) speed_pid.out = -800.0f;
    
    return speed_pid.out;
}