#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include <string.h>
#include "motor_types.h"

// 定义CAN接收数据结构体
typedef struct {
    uint16_t id;           // 接收的ID
    uint8_t data[8];      // 接收的数据
    uint8_t len;           // 数据长度
} CANRxData;

// 定义回调函数指针类型
typedef void (*can_rx_callback_t)(CANRxData *rx_data, CAN_PORT can_port);

void bsp_can1_set_callback(can_rx_callback_t callback);
void bsp_can2_set_callback(can_rx_callback_t callback);
void bsp_can3_set_callback(can_rx_callback_t callback);
void bsp_can_init(void);
void can_filter_init(FDCAN_HandleTypeDef *hfdcan, uint8_t fifo);
void can_filter_ext_init(FDCAN_HandleTypeDef *hfdcan, uint8_t fifo);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_ext_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);

#endif 

