#ifndef __OPEN_MV_H_
#define __OPEN_MV_H_

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
void USART3_openmv_Init(u32 bound);
void USART3_IRQHandler(void);
void uart4_init(u32 bound);
void uart5_init(u32 bound);

//--------------serial--------------//
void Serial_SendByte(uint8_t Byte,USART_TypeDef* USART);
void Serial_SendArray(uint8_t *Array, uint16_t Length,USART_TypeDef* USART);
void Serial_SendString(char *String,USART_TypeDef* USART);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void Serial_SendNumber(uint32_t Number, uint8_t Length,USART_TypeDef* USART);
int fputc4(int ch, FILE *f);
int fputc5(int ch, FILE *f);
void Serial_Printf4(char *format, ...);

extern uint8_t trace_num,trace_tail,turn,trace_array[5];
extern uint8_t flag_start,data[50],turn_flag;

void processOpenMVData(uint8_t traceData, uint8_t turnFlag);

#endif