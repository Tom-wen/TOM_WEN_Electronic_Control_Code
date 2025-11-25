#include "stm32f10x.h"                  // Device header
#include "SERVO.h"
#include "PWM.h"

void servo_init()
{
	PWM_SERVO_Init();
}

/**
 * @brief 设置舵机角度
 * @param Angle 舵机角度，范围：0~180度
 * @note 占空比计算公式：500~2500us 对应 0~180度
 */
void Servo_SetAngle(float Angle)
{
    // 角度限幅保护
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    
    // 线性映射：角度 -> 脉冲宽度(500~2500us)
    float pulse_width = (Angle / 180.0f) * 2000.0f + 500.0f;
    PWM_Set_SERVO_Compare(pulse_width);
}