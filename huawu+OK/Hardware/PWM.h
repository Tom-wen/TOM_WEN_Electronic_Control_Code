#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare3(uint16_t Compare);  // PB0 - TIM3_CH3
void PWM_SetCompare4(uint16_t Compare);  // PB1 - TIM3_CH4
void test_motor_left(uint16_t Compare);
void test_motor_right(uint16_t Compare);

//SERVO ¶æ»ú
void PWM_Set_SERVO_Compare(uint16_t Compare);
void PWM_SERVO_Init(void);

#endif
