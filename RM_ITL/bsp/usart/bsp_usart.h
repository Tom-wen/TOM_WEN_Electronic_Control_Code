#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

//大疆遥控器
//#define DJI_REMOTE
//富斯遥控器
//#define FS_REMOTE
//图传遥控器
#define VT_03_REMOTE

/* 二、定义串口数据缓冲区配置 */
#define UART_BUFFER_SIZE        (128)
#define UART_BUFFER_QUANTITY    (5)

/* 三、定义串口数据结构体，DMA传输的数据将保存在这里 */
typedef struct
{
    uint8_t buffer[UART_BUFFER_SIZE];   /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART_RX_TypeDef;

//遥控器状态检测使用
extern volatile uint8_t sbus_online;
extern volatile uint32_t last_sbus_recv_time;

// 定义回调函数指针类型
typedef void (*usart_rx_callback_t)(uint8_t *sbus_buf);

extern uint8_t usart1_rx_buf[256];
extern uint8_t usart2_rx_buf[256];
extern uint8_t usart7_rx_buf[256];
extern uint8_t sbus_buf[25];
extern uint8_t usart10_rx_buf[256];

extern uint8_t uart3_buff_ctrl;
extern UART_RX_TypeDef uart3_rx_data_t[UART_BUFFER_QUANTITY];

void bsp_usart5_set_callback(usart_rx_callback_t callback);
void bsp_usart7_set_callback(usart_rx_callback_t callback);
uint8_t usart_tx_dma_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
uint8_t usart_rx_dma_start(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);
void usart_rx_data_process(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);
void usart_vofa_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
void Upper_Computer_Init(float* addr);
void usart_vofa_send(UART_HandleTypeDef *huart);
// 定义数据结构体
typedef struct {
    float voltage;
    float current;
    float power;
    float energy;
} PowerMonitor_Data_t;
void power_data_clac(uint8_t *buf, PowerMonitor_Data_t *power_data);
extern PowerMonitor_Data_t power_data;
#endif
