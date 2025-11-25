#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_LEFT_FORWARD GPIO_Pin_0
#define MOTOR_LEFT_BACKWARD GPIO_Pin_1
#define MOTOR_RIGHT_FORWARD GPIO_Pin_2
#define MOTOR_RIGHT_BACKWARD GPIO_Pin_3
#define MOTOR_LEFT_GPIO GPIOC
#define MOTOR_RIGHT_GPIO GPIOC

void Motor_Init(void);
void Motor_SetSpeed(int8_t Speed);
void set_motor_speeds(float left_speed, float right_speed);
void Motor_MECNAMU_SetSpeed(float chassis_vx,float chassis_vy);



#endif
