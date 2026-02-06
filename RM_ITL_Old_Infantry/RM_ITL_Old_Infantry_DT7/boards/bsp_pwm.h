#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "main.h"
#include "stdint.h"
#include "tim.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#endif
