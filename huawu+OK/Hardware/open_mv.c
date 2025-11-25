#include "stm32f10x.h"                  // Device header
#include "OPEN_MV.h"
#include <stdio.h>
#include <stdarg.h>

//uint8_t crossroad=0,stop_flag=0,Turn=0,num=0,flag_start=0;
uint8_t data[50];
uint8_t trace_num,trace_tail,turn,trace_array[5];
uint8_t flag_start,turn_flag;
#define RX_BUFFER_SIZE 10
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;
uint8_t receiving = 0;
uint8_t openmv_trace = 0;
uint8_t openmv_turn = 0;

void USART3_openmv_Init(u32 bound)
{
    /*????*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    /*GPIO?? - ??*/
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PD8 - USART3_TX ??????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PD9 - USART3_RX ????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*USART??*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    
    /*??????*/
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    
    /*NVIC??*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // ?????
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /*????*/
    USART_Cmd(USART3, ENABLE);
    }
void processOpenMVData(uint8_t traceData, uint8_t turnFlag)
{
    // ??????(5??????)
    uint8_t trace[5];
    for(int i = 0; i < 5; i++)
		{
        trace[i] = (traceData >> i) & 0x01;
    }
    // ??????(??)
    printf("Trace: %d%d%d%d%d, Turn: %d\n", 
           trace[4], trace[3], trace[2], trace[1], trace[0], turnFlag);
    
    // ??????????
//    controlMotor(trace, turnFlag);
}


void USART3_IRQHandler(void)
{
	static int j=0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)		//?????USART1??????????
	{
		data[j++]=USART_ReceiveData(USART3);
		if(flag_start == 0)
		{
			if(data[0]!=0xA3) j=0;
			if(j==4)
			{
				if(data[3]==0xC3)
				{
					flag_start = 1;
				}
				j=0;
			}
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);		//?????
		}
		if(flag_start == 1)
		{	
			if(data[0]!=0xA3) j=0;
			if(j==4)
			{
				if(data[3]==0xC3)
				{
					trace_num=data[1];         //??????????,????????,?????????
					turn=data[2];        //???????,??????????????,???????
				}
				j=0;
			}
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);		//?????
		}
	}
}

void uart4_init(u32 bound)
{
  //GPIO????
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	// GPIOC??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE); //??4????
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//??????
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????
  GPIO_Init(GPIOC, &GPIO_InitStructure);//???GPIOA.10  
 
  //Usart1 NVIC ??
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//?????3 ???????
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC???
  
   //USART ?????
 
	USART_InitStructure.USART_BaudRate = bound;//?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
 
  USART_Init(UART4, &USART_InitStructure); //?????1
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//????????
  USART_Cmd(UART4, ENABLE);                    //????1 
 
}

void uart5_init(u32 bound)
{
  //GPIO????
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);// GPIOC??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE); //??4????
	
  
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//??????
  GPIO_Init(GPIOC, &GPIO_InitStructure);//???GPIOA.9
   
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
 
  //Usart1 NVIC ??
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//?????3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC???
  
   //USART ?????
 
	USART_InitStructure.USART_BaudRate = bound;//?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
 
  USART_Init(UART5, &USART_InitStructure); //?????1
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//????????
  USART_Cmd(UART5, ENABLE);                    //????1 
 
}



void Serial_SendByte(uint8_t Byte,USART_TypeDef* USART)
{
	USART_SendData(USART, Byte);
	while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length,USART_TypeDef* USART)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i],USART);
	}
}

void Serial_SendString(char *String,USART_TypeDef* USART)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i],USART);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length,USART_TypeDef* USART)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0',USART);
	}
}


int fputc4(int ch, FILE *f)
{
	Serial_SendByte(ch,UART4);			//?printf????????????????
	return ch;
}

int fputc5(int ch, FILE *f)
{
	Serial_SendByte(ch,UART5);			//?printf????????????????
	return ch;
}

void Serial_Printf4(char *format, ...)
{
	char String[100];				//??????
	va_list arg;					//???????????????arg
	va_start(arg, format);			//?format??,???????arg??
	vsprintf(String, format, arg);	//??vsprintf???????????????????
	va_end(arg);					//????arg
	Serial_SendString(String,UART4);		//????????(???)
}




void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//?????USART1??????????
	{	
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);			//??USART1?RXNE???
	}
}	
															



void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
}
void UART5_IRQHandler(void)
{
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
}

