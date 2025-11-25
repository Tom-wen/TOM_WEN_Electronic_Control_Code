#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init();
int16_t Encoder_Get();
void All_Encoders_Init(void);

// 编码器类型定义
typedef enum {
    ENCODER_LEFT_REAR = 0,   // 左后电机 - TIM8 PC6 PC7
    ENCODER_LEFT_FRONT,      // 左前电机 - TIM1 PA8 PA9  
    ENCODER_RIGHT_FRONT      // 右前电机 - TIM4 PB6 PB7
} Encoder_Type;

#endif
