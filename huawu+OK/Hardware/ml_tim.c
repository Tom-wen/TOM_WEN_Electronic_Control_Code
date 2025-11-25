#include "headfile.h"

TIM_TypeDef *tim_index[3] = { TIM2 , TIM3 , TIM4 };

//-------------------------------------------------------------------------------------------------------------------
// @brief		内部定时器初始化(同时初始化定时器中断)
// @param	  timn		选择定时器(可选用的定时器参考ml_tim.h中的枚举定义)
// @param	  time_ms  进入定时器中断的间隔(ms)
// @param	  priority 设置中断优先级(0~15 越小优先级越高)
// @return		void  
// Sample usage:	tim_interrupt_ms_init(TIM_2,1000,0);
//-------------------------------------------------------------------------------------------------------------------
void tim_interrupt_ms_init(TIMn_enum timn,int time_ms,uint8_t priority)
{	
		RCC->APB1ENR |= 1<<timn;  //定时器时钟使能

	  tim_index[timn]->ARR = 10*time_ms-1; //自动重装载值
	  tim_index[timn]->PSC = 7200-1;       //预分频器值
	  tim_index[timn]->CR1 |= 0x01;        //使能计数器
    tim_index[timn]->DIER |= 0x01;       //使能定时器中断
	  
		NVIC_init(priority,timn+28);         //中断管理初始化

}


void Timer1_Init(u16 ms,uint8_t Pre_Priority)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
 
  TIM_TimeBaseStructure.TIM_Period = 10*ms-1; //设置自动重装载寄存器周期值
  TIM_TimeBaseStructure.TIM_Prescaler =7200-1;//设置预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//重复计数设置
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //参数初始化
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清中断标志位
 
  TIM_ITConfig(      //使能或者失能指定的TIM中断
    TIM1,            //TIM1
    TIM_IT_Update  | //TIM 更新中断源
    TIM_IT_Trigger,  //TIM 触发中断源 
    ENABLE  	     //使能
    );
	
  //设置优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Pre_Priority;//先占优先级0级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  	   //从优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
 
  TIM_Cmd(TIM1, ENABLE);  //使能TIMx外设
}


