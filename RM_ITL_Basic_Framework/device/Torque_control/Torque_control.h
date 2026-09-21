#ifndef __TORQUE_CONTROL_H__
#define __TORQUE_CONTROL_H__

#include "Init.h"

// 动力学模型参数
#define Motor_Kt    0.741f  // 6020电机扭矩常数 (N·m/A)
#define Motor_J     0.04436f  // 奶龙步兵云台惯量（kg·m²）
#define Motor_B     0.00410f    //奶龙步兵阻尼系数(N·m/(rad/s))

extern uint8_t sweep_running;
extern float torque;
// 前馈计算（返回raw电流，可直接叠加到target_current）
float Yaw_Torque_Control(float omega, float alpha);

// 扫频辨识
void     Yaw_Sweep_Start(void);
void     Yaw_Sweep_Stop(void);
uint8_t  Yaw_Sweep_IsRunning(void);
void     Yaw_Sweep_Update(MotorInstance *motor, UART_HandleTypeDef *huart,
                          float omega, float alpha);

#endif
