#include "bsp_buzzer.h"

void buzzer_init() 
{ 
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    htim12.Instance->ARR = 40000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(250);  
    
    htim12.Instance->ARR = 28000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(250);      

    htim12.Instance->ARR = 21000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(300);  

    htim12.Instance->CCR2 = 0;
}

void buzzer_warning()
{
    while(1)
    {
        htim12.Instance->CCR2 =500;
        HAL_Delay(300);
        htim12.Instance->CCR2 =0;
        HAL_Delay(300);
    }
}
