#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include <string.h>

#define MAX_CAN_CALLBACKS 5

// 定义CAN口
typedef enum
{
    CAN1 = 0,
    CAN2 = 1,
    CAN3 = 2,
    CAN4 = 3,  // MCP2515 #1 SPI转CAN
    CAN5 = 4,  // MCP2515 #2 SPI转CAN
    CAN6 = 5   // MCP2515 #3 SPI转CAN
} CAN_PORT;

// MCP2515 虚拟 CAN 句柄（用作标识，不是真正的 FDCAN 外设）
extern FDCAN_HandleTypeDef hfdcan4_mcp2515;
extern FDCAN_HandleTypeDef hfdcan5_mcp2515;
extern FDCAN_HandleTypeDef hfdcan6_mcp2515;

// 定义CAN接收数据结构体
typedef struct {
    uint16_t id;           // 接收的ID
    uint8_t data[8];      // 接收的数据
    uint8_t len;           // 数据长度
} CANRxData;

// 定义回调函数指针类型
typedef void (*can_rx_callback_t)(CANRxData *rx_data, CAN_PORT can_port);

void bsp_can1_register_callback(can_rx_callback_t cb);
void bsp_can2_register_callback(can_rx_callback_t cb);
void bsp_can3_register_callback(can_rx_callback_t cb);
void bsp_can4_register_callback(can_rx_callback_t cb);
void bsp_can5_register_callback(can_rx_callback_t cb);
void bsp_can6_register_callback(can_rx_callback_t cb);
void bsp_can_init(void);
void can_filter_init(FDCAN_HandleTypeDef *hfdcan, uint8_t fifo);
void can_filter_ext_init(FDCAN_HandleTypeDef *hfdcan, uint8_t fifo);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
void fdcan4_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len);
void fdcan5_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len);
void fdcan6_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);

#endif
