#include "stm32f10x.h"
#include "misc.h"

void Timer_Init(int ARR,int PSC)
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /*配置时钟源*/
    TIM_InternalClockConfig(TIM8);
    
    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;        // ARR = 10000
    TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;      // PSC = 7200
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    /*清除更新标志位 - 重要！*/
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    
    /*使能更新中断*/
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    /*NVIC中断分组 - 在main函数开始处只调用一次*/
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 移到main函数
    
    /*NVIC配置*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    /*TIM使能*/
    TIM_Cmd(TIM4, ENABLE);
}
//void Timer_Init(int ARR,int PSC)
//{
//    /*开启时钟*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//    
//    /*配置时钟源*/
//    TIM_InternalClockConfig(TIM5);
//    
//    /*时基单元初始化*/
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;        // ARR = 10000
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;      // PSC = 7200
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
//    
//    /*清除更新标志位 - 重要！*/
//    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
//    
//    /*使能更新中断*/
//    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
//    
//    /*NVIC中断分组 - 在main函数开始处只调用一次*/
//     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 移到main函数
//    
//    /*NVIC配置*/
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_Init(&NVIC_InitStructure);
//    
//    /*TIM使能*/
//    TIM_Cmd(TIM5, ENABLE);
//}