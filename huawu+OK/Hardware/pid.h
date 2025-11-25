#ifndef __PID_h_
#define __PID_h_

#include "headfile.h"

enum
{
    POSITION_PID = 0,
    DELTA_PID,
};

typedef struct
{
    float target;    
    float now;
    float error[3];        
    float p, i, d;
    float pout, dout, iout;
    float out;   
    uint32_t pid_mode;
} pid_t;

// 巡线PID参数
extern float line_kp, line_ki, line_kd;
extern float speed_kp, speed_ki, speed_kd;

// PID实例
extern pid_t line_pid;    // 巡线PID
extern pid_t speed_pid;   // 速度PID

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_t *pid);
void pid_reset(pid_t *pid);
extern float line_pid_control(float current_position);
extern float speed_pid_control(float current_speed, float target_speed);

#endif
