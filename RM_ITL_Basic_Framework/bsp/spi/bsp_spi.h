#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "main.h"
#include "spi.h"


// MCP2515 寄存器定义
#define MCP2515_WRITE_CMD 0x02
#define MCP2515_READ_CMD 0x03
#define MCP2515_BITMOD_CMD 0x05
#define MCP2515_LOAD_TX0_CMD 0x40
#define MCP2515_RTS_CMD 0x80

// CAN 初始化函数
void MCP2515_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// 发送 CAN 帧函数
void MCP2515_Send(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t id, uint8_t *data, uint8_t len);
// 接收 CAN 帧函数
void MCP2515_Receive(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *id, uint8_t *data, uint8_t *len);
// 清除 RX0IF
void MCP2515_RX0IF_clear(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// 接收轮询
void MCP2515_Rx_Poll(void);
void MCP2515_2_Rx_Poll(void);
void MCP2515_3_Rx_Poll(void);

// 三路 MCP2515 接收中断标志（ISR 置位，poll 读取）
extern volatile uint8_t mcp2515_1_rx_flag;
extern volatile uint8_t mcp2515_2_rx_flag;
extern volatile uint8_t mcp2515_3_rx_flag;

#endif
