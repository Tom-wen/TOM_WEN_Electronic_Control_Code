#ifndef __USART_USER_H_
#define __USART_USER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"
#include "string.h"
#include "usart.h"
#include "GQ_Motor.h"



#define BUFFER_SIZE 256  
extern uint8_t received_buffer[BUFFER_SIZE];
void user_usart_init(void);
void process_received_data(uint8_t *buf, uint16_t this_time_rx_len);
void VOFA_Send_Float_Data(float ch1, float ch2, float ch3, float ch4);
	
	
	
	
	
#ifdef __cplusplus
}
#endif

#endif
	