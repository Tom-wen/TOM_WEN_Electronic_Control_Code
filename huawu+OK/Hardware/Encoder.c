#include "stm32f10x.h"
#include "Delay.h"
#include "Encoder.h"

/**
  * 函    数：编码器初始化
  * 参    数：encoder - 编码器类型
  * 返 回 值：无
  */
void Encoder_Init(Encoder_Type encoder)
{
    TIM_TypeDef* TIMx;
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin1, GPIO_Pin2;
    uint32_t RCC_APBPeriph_TIM, RCC_APB2Periph_GPIO;
    
    switch(encoder) {
        case ENCODER_LEFT_REAR:   // TIM8 PC6 PC7
            TIMx = TIM8;
            GPIOx = GPIOC;
            GPIO_Pin1 = GPIO_Pin_6;
            GPIO_Pin2 = GPIO_Pin_7;
            RCC_APBPeriph_TIM = RCC_APB2Periph_TIM8;
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOC;
            break;
            
        case ENCODER_LEFT_FRONT:  // TIM1 PA8 PA9
            TIMx = TIM1;
            GPIOx = GPIOA;
            GPIO_Pin1 = GPIO_Pin_8;
            GPIO_Pin2 = GPIO_Pin_9;
            RCC_APBPeriph_TIM = RCC_APB2Periph_TIM1;
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOA;
            break;
            
        case ENCODER_RIGHT_FRONT: // TIM4 PB6 PB7
            TIMx = TIM4;
            GPIOx = GPIOB;
            GPIO_Pin1 = GPIO_Pin_6;
            GPIO_Pin2 = GPIO_Pin_7;
            RCC_APBPeriph_TIM = RCC_APB1Periph_TIM4;
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOB;
            break;
            
        default:
            return;
    }
    
    /*开启时钟 - 修正后的判断逻辑*/
    if (TIMx == TIM1 || TIMx == TIM8) {
        RCC_APB2PeriphClockCmd(RCC_APBPeriph_TIM, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(RCC_APBPeriph_TIM, ENABLE);
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO, ENABLE);
    
    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin1 | GPIO_Pin2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
    
    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);
    
    /*输入捕获初始化*/
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    
    /*编码器接口配置*/
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    /*TIM使能*/
    TIM_Cmd(TIMx, ENABLE);
    
    /*高级定时器需要额外使能主输出*/
    if (TIMx == TIM1 || TIMx == TIM8) {
        TIM_CtrlPWMOutputs(TIMx, ENABLE);
    }
}

/**
  * 函    数：获取编码器的增量值
  * 参    数：encoder - 编码器类型
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Encoder_Get(Encoder_Type encoder)
{
    TIM_TypeDef* TIMx;
    
    // 根据编码器类型选择定时器
    switch(encoder) {
        case ENCODER_LEFT_REAR:
            TIMx = TIM8;
            break;
        case ENCODER_LEFT_FRONT:
            TIMx = TIM1;
            break;
        case ENCODER_RIGHT_FRONT:
            TIMx = TIM4;
            break;
        default:
            return 0; // 无效的编码器类型
    }
    
    /*使用Temp变量作为中继，目的是返回CNT后将其清零*/
    uint16_t Temp;
    Temp = TIM_GetCounter(TIMx);
    TIM_SetCounter(TIMx, 0);
    return Temp;
}

/**
  * 函    数：初始化所有编码器
  * 参    数：无
  * 返 回 值：无
  */
void All_Encoders_Init(void)
{
    Encoder_Init(ENCODER_LEFT_REAR);
    Encoder_Init(ENCODER_LEFT_FRONT);
    Encoder_Init(ENCODER_RIGHT_FRONT);
}

/**
  * 函    数：获取所有编码器的值
  * 参    数：leftRear - 左后编码器值指针
  *         leftFront - 左前编码器值指针  
  *         rightFront - 右前编码器值指针
  * 返 回 值：无
  */
void All_Encoders_Get(int16_t* leftRear, int16_t* leftFront, int16_t* rightFront)
{
    if(leftRear) *leftRear = Encoder_Get(ENCODER_LEFT_REAR);
    if(leftFront) *leftFront = Encoder_Get(ENCODER_LEFT_FRONT);
    if(rightFront) *rightFront = Encoder_Get(ENCODER_RIGHT_FRONT);
}

//int16_t Encoder_Count;

//void Encoder_Init(void)
//{
//    /*开启时钟*/
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//    
//    /*GPIO初始化*/
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    
//    /*AFIO选择中断引脚*/
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
//    
//    /*EXTI初始化*/
//    EXTI_InitTypeDef EXTI_InitStructure;
//    EXTI_InitStructure.EXTI_Line = EXTI_Line0| EXTI_Line1;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//    EXTI_Init(&EXTI_InitStructure);
//    
//    /*NVIC中断分组*/
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    
//    /*NVIC配置*/
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_Init(&NVIC_InitStructure);
//}

//int16_t Encoder_Get(void)
//{
//    int16_t Temp;
//    Temp = Encoder_Count;
//    Encoder_Count = 0;
//    return Temp;
//}

//// 改进的方向判断逻辑
//void EXTI0_IRQHandler(void)
//{
//    if (EXTI_GetITStatus(EXTI_Line0) == SET)
//    {
//        // 添加延时消抖
//        Delay_us(100);  // 延时100微秒
//        
//        // 确认PA0确实为低电平
//        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
//        {
//            // 读取PA1的电平状态来判断方向
//            if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)
//            {
//                // PA0下降沿时，PA1为高电平，表示正转
//                Encoder_Count++;
//            }
//            else
//            {
//                // PA0下降沿时，PA1为低电平，表示反转
//                Encoder_Count--;
//            }
//        }
//        EXTI_ClearITPendingBit(EXTI_Line0);
//    }
//}

//void EXTI1_IRQHandler(void)
//{
//    if (EXTI_GetITStatus(EXTI_Line1) == SET)
//    {
//        // 添加延时消抖
//        Delay_us(100);  // 延时100微秒
//        
//        // 确认PA1确实为低电平
//        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
//        {
//            // 读取PA0的电平状态来判断方向
//            if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)
//            {
//                // PA1下降沿时，PA0为高电平，表示反转
//                Encoder_Count--;
//            }
//            else
//            {
//                // PA1下降沿时，PA0为低电平，表示正转
//                Encoder_Count++;
//            }
//        }
//        EXTI_ClearITPendingBit(EXTI_Line1);
//    }
//}

///**
//  * 函    数：编码器初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
//左后电机 TIM8 PC6 CH1 PC7-CH2
//void Encoder_Init(void)
//{
//	/*开启时钟*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);			//开启TIM3的时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);			//开启GPIOA的时钟
//	
//	/*GPIO初始化*/
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);							//将PA6和PA7引脚初始化为上拉输入
//	
//	/*时基单元初始化*/
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
//	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
//	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
//	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
//	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                //预分频器，即PSC的值
//	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
//	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
//	
//	/*输入捕获初始化*/
//	TIM_ICInitTypeDef TIM_ICInitStructure;							//定义结构体变量
//	TIM_ICStructInit(&TIM_ICInitStructure);							//结构体初始化，若结构体没有完整赋值
//																	//则最好执行此函数，给结构体所有成员都赋一个默认值
//																	//避免结构体初值不确定的问题
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				//选择配置定时器通道1
//	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
//	TIM_ICInit(TIM8, &TIM_ICInitStructure);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;				//选择配置定时器通道2
//	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
//	TIM_ICInit(TIM8, &TIM_ICInitStructure);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
//	
//	/*编码器接口配置*/
//	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//																	//配置编码器模式以及两个输入通道是否反相
//																	//注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
//																	//此函数必须在输入捕获初始化之后进行，否则输入捕获的配置会覆盖此函数的部分配置
//	
//	/*TIM使能*/
//	TIM_Cmd(TIM8, ENABLE);			//使能TIM3，定时器开始运行
//}

///**
//  * 函    数：获取编码器的增量值
//  * 参    数：无
//  * 返 回 值：自上此调用此函数后，编码器的增量值
//  */
//int16_t Encoder_Get(void)
//{
//	/*使用Temp变量作为中继，目的是返回CNT后将其清零*/
//	int16_t Temp;
//	Temp = TIM_GetCounter(TIM8);
//	TIM_SetCounter(TIM8, 0);
//	return Temp;
//}

///**
//  * 函    数：编码器初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
////左前电机 TIM1  PA8-CH1  PA9- CH2
//void Encoder_Init(void)
//{
///*开启时钟*/
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    
//    /*GPIO初始化 - 使用PC6和PC7*/
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;  // PC6=TIM8_CH1, PC7=TIM8_CH2
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    
//    /*时基单元初始化*/
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
//    
//    /*编码器接口配置*/
//    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//    
//    /*TIM使能*/
//    TIM_Cmd(TIM1, ENABLE);
//}

///**
//  * 函    数：获取编码器的增量值
//  * 参    数：无
//  * 返 回 值：自上此调用此函数后，编码器的增量值
//  */
//int16_t Encoder_Get(void)
//{
//	/*使用Temp变量作为中继，目的是返回CNT后将其清零*/
//	int16_t Temp;
//	Temp = TIM_GetCounter(TIM1);
//	TIM_SetCounter(TIM1, 0);
//	return Temp;
//}

///**
//  * 函    数：编码器初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
////右前电机 TIM4 PB6-CH1 PB7-CH2
//void Encoder_Init(void)
//{
///*开启时钟*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    
//    /*GPIO初始化 - 使用PC6和PC7*/
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  // PC6=TIM8_CH1, PC7=TIM8_CH2
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//    
//    /*时基单元初始化*/
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
//    
//    /*编码器接口配置*/
//    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//    
//    /*TIM使能*/
//    TIM_Cmd(TIM4, ENABLE);
//}

///**
//  * 函    数：获取编码器的增量值
//  * 参    数：无
//  * 返 回 值：自上此调用此函数后，编码器的增量值
//  */
//int16_t Encoder_Get(void)
//{
//	/*使用Temp变量作为中继，目的是返回CNT后将其清零*/
//	int16_t Temp;
//	Temp = TIM_GetCounter(TIM4);
//	TIM_SetCounter(TIM4, 0);
//	return Temp;
//}

///**
//  * 函    数：编码器初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
//void Encoder_Init(void)
//{
//    /*开启时钟*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    // 确保AFIO时钟开启
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	
//    /*GPIO初始化 - 使用PA0和PA1*/
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    
//    /*时基单元初始化*/
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 65535;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
//    
//    /*输入捕获通道配置*/
//    TIM_ICInitTypeDef TIM_ICInitStructure;
//    TIM_ICStructInit(&TIM_ICInitStructure);
//    TIM_ICInitStructure.TIM_ICFilter = 0;  // 先不滤波，看是否有计数
//    
//    // 通道1
//    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//    TIM_ICInit(TIM5, &TIM_ICInitStructure);
//    
//    // 通道2
//    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//    TIM_ICInit(TIM5, &TIM_ICInitStructure);
//    
//    /*编码器接口配置：尝试不同的模式，比如TI1模式*/
//    // TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//    
//    /*TIM使能*/
//    TIM_Cmd(TIM5, ENABLE);
//}
///**
//  * 函    数：获取编码器的增量值
//  * 参    数：无
//  * 返 回 值：自上此调用此函数后，编码器的增量值
//  */
//int32_t Encoder_Get(void)  // 改为int32_t以匹配32位定时器
//{
//    int32_t Temp;
//    Temp = (int32_t)TIM_GetCounter(TIM5);
//    TIM_SetCounter(TIM5, 0);
//    return Temp;
//}
