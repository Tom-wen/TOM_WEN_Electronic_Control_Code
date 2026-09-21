#include "bsp_can.h"
#include "bsp_spi.h"
#include "detect.h"
#include "config.h"
#include "dm_imu.h"
#include "FreeRTOS.h"
#include "task.h"

uint8_t FreeLevel1 = 0;
uint8_t check = 0;

// MCP2515 虚拟 CAN 句柄
FDCAN_HandleTypeDef hfdcan4_mcp2515 = {0};
FDCAN_HandleTypeDef hfdcan5_mcp2515 = {0};
FDCAN_HandleTypeDef hfdcan6_mcp2515 = {0};

// 每个CAN口的回调列表
static can_rx_callback_t can1_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can1_cb_count = 0;
static can_rx_callback_t can2_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can2_cb_count = 0;
static can_rx_callback_t can3_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can3_cb_count = 0;
static can_rx_callback_t can4_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can4_cb_count = 0;
static can_rx_callback_t can5_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can5_cb_count = 0;
static can_rx_callback_t can6_callbacks[MAX_CAN_CALLBACKS];
static uint8_t can6_cb_count = 0;

// 为每个CAN口设置独立的回调函数
//CAN1
void bsp_can1_register_callback(can_rx_callback_t cb)
{
    if (!cb || can1_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can1_cb_count; i++){
        if (can1_callbacks[i] == cb) return; // 已注册，跳过
    }
    can1_callbacks[can1_cb_count++] = cb;
}
//CAN2
void bsp_can2_register_callback(can_rx_callback_t cb)
{
    if (!cb || can2_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can2_cb_count; i++){
        if (can2_callbacks[i] == cb) return;
    }
    can2_callbacks[can2_cb_count++] = cb;
}
//CAN3
void bsp_can3_register_callback(can_rx_callback_t cb)
{
    if (!cb || can3_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can3_cb_count; i++){
        if (can3_callbacks[i] == cb) return;
    }
    can3_callbacks[can3_cb_count++] = cb;
}
//CAN4 (MCP2515 #1)
void bsp_can4_register_callback(can_rx_callback_t cb)
{
    if (!cb || can4_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can4_cb_count; i++){
        if (can4_callbacks[i] == cb) return;
    }
    can4_callbacks[can4_cb_count++] = cb;
}
//CAN5 (MCP2515 #2)
void bsp_can5_register_callback(can_rx_callback_t cb)
{
    if (!cb || can5_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can5_cb_count; i++){
        if (can5_callbacks[i] == cb) return;
    }
    can5_callbacks[can5_cb_count++] = cb;
}
//CAN6 (MCP2515 #3)
void bsp_can6_register_callback(can_rx_callback_t cb)
{
    if (!cb || can6_cb_count >= MAX_CAN_CALLBACKS) return;
    for (uint8_t i = 0; i < can6_cb_count; i++){
        if (can6_callbacks[i] == cb) return;
    }
    can6_callbacks[can6_cb_count++] = cb;
}

//bsp_can初始化
void bsp_can_init(void)
{
	can_filter_init(&hfdcan1, 0);
    HAL_FDCAN_Start(&hfdcan1);
    can_filter_init(&hfdcan2, 1);
    HAL_FDCAN_Start(&hfdcan2);
    can_filter_init(&hfdcan3, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

//标准can滤波器配置,fifo为1是FIFO1,fifo为0是FIFO0
void can_filter_init(FDCAN_HandleTypeDef *hfdcan, uint8_t fifo)
{
    FDCAN_FilterTypeDef fdcan_filter;

    switch (fifo)
    {
    case 1:
        fdcan_filter.IdType = FDCAN_STANDARD_ID;
        fdcan_filter.FilterIndex = 0;
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        fdcan_filter.FilterID1 = 0x00;
        fdcan_filter.FilterID2 = 0x00;

        HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        break;
    case 0:
        fdcan_filter.IdType = FDCAN_STANDARD_ID;
        fdcan_filter.FilterIndex = 0;
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        fdcan_filter.FilterID1 = 0x00;
        fdcan_filter.FilterID2 = 0x00;

        HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        break;
    }

}
//标准帧can发送
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    // MCP2515 SPI转CAN
    if (hfdcan == &hfdcan4_mcp2515) {
        taskENTER_CRITICAL();
        MCP2515_Send(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, id, data, len);
        taskEXIT_CRITICAL();
        return 1;
    }
    else if (hfdcan == &hfdcan5_mcp2515) {
        taskENTER_CRITICAL();
        MCP2515_Send(&hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, id, data, len);
        taskEXIT_CRITICAL();
        return 1;
    }
    else if (hfdcan == &hfdcan6_mcp2515) {
        taskENTER_CRITICAL();
        MCP2515_Send(&hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin, id, data, len);
        taskEXIT_CRITICAL();
        return 1;
    }

    FDCAN_TxHeaderTypeDef pTxHeader;
    if(id > 0x7FF)
    {
        pTxHeader.Identifier=id;
        pTxHeader.IdType=FDCAN_EXTENDED_ID;
    }
    else
    {
        pTxHeader.Identifier=id;
        pTxHeader.IdType=FDCAN_STANDARD_ID;
    }
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	pTxHeader.DataLength =FDCAN_DLC_BYTES_8;
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;

    FreeLevel1 = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);

	while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 0)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data);
		return 1;
	}
    return 0;
}

//can1接收函数
void fdcan1_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t rx_data[8];
    if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, rx_data) == HAL_OK)
    {
        CANRxData rx_tmp;
        rx_tmp.id = pRxHeader.Identifier;
        rx_tmp.len = 8;
        memcpy(rx_tmp.data, rx_data, 8);
        for(uint8_t i = 0; i < can1_cb_count; i++){
            can1_callbacks[i](&rx_tmp, CAN1);
        }
    }
}

//can2接收函数
void fdcan2_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t rx_data[8];

    if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &pRxHeader, rx_data) == HAL_OK)
    {
        CANRxData rx_tmp;
        rx_tmp.id = pRxHeader.Identifier;
        rx_tmp.len = 8;
        memcpy(rx_tmp.data, rx_data, 8);
        if(pRxHeader.Identifier == 0x212){
            rx_tmp.len = 4;
            power_data_callback(&rx_tmp, CAN2);
        }
        else if(pRxHeader.Identifier == 0x0FF){
            super_cap_callback(&rx_tmp, CAN2);
        }
        #ifdef ROBOTIC_SWING_CHASSIS
        else if(pRxHeader.Identifier == 0x2A8)
        {
            IMU_UpdateData(rx_data);
        }
        #endif
        else{
            for(uint8_t i = 0; i < can2_cb_count; i++){
                can2_callbacks[i](&rx_tmp, CAN2);
                
            }
        }
    }
}

//can3接收函数
void fdcan3_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t rx_data[8];

    if(HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &pRxHeader, rx_data) == HAL_OK)
    {
        CANRxData rx_tmp;
        rx_tmp.id = pRxHeader.Identifier;
        rx_tmp.len = 8;
        memcpy(rx_tmp.data, rx_data, 8);
        for(uint8_t i = 0; i < can3_cb_count; i++){
            can3_callbacks[i](&rx_tmp, CAN3);
        }
    }
}

// CAN4 (MCP2515 #1) 接收分发
void fdcan4_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len)
{
    CANRxData rx_tmp;
    rx_tmp.id = id;
    rx_tmp.len = len;
    memcpy(rx_tmp.data, data, len);
    for(uint8_t i = 0; i < can4_cb_count; i++){
        can4_callbacks[i](&rx_tmp, CAN4);
    }
}

// CAN5 (MCP2515 #2) 接收分发
void fdcan5_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len)
{
    CANRxData rx_tmp;
    rx_tmp.id = id;
    rx_tmp.len = len;
    memcpy(rx_tmp.data, data, len);
    for(uint8_t i = 0; i < can5_cb_count; i++){
        can5_callbacks[i](&rx_tmp, CAN5);
    }
}

// CAN6 (MCP2515 #3) 接收分发
void fdcan6_mcp2515_rx_dispatch(uint32_t id, uint8_t *data, uint8_t len)
{
    CANRxData rx_tmp;
    rx_tmp.id = id;
    rx_tmp.len = len;
    memcpy(rx_tmp.data, data, len);
    for(uint8_t i = 0; i < can6_cb_count; i++){
        can6_callbacks[i](&rx_tmp, CAN6);
    }
}

//CANFIFO0中断回调函数 — CAN1和CAN3共用FIFO0
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
    {
        fdcan1_rx_callback();
    }
    else if(hfdcan == &hfdcan3)
    {
        fdcan3_rx_callback();
    }
}

//CANFIFO1中断回调函数 — CAN2独占FIFO1
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan == &hfdcan2)
    {
        fdcan2_rx_callback();
    }
}

//CAN通信错误处理
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if(hfdcan == &hfdcan1 || hfdcan == &hfdcan3)
    {
        error_code = can_busy;
        FDCAN_FilterTypeDef fdcan_filter;
        fdcan_filter.IdType = FDCAN_STANDARD_ID;
        fdcan_filter.FilterIndex = 0;
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        fdcan_filter.FilterID1 = 0x00;
        fdcan_filter.FilterID2 = 0x00;
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_DeInit(hfdcan);
        HAL_FDCAN_Init(hfdcan);
        HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(hfdcan);
    }
    else if(hfdcan == &hfdcan2)
    {
        error_code = can_busy;
        FDCAN_FilterTypeDef fdcan_filter;
        fdcan_filter.IdType = FDCAN_STANDARD_ID;
        fdcan_filter.FilterIndex = 0;
        fdcan_filter.FilterType = FDCAN_FILTER_MASK;
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        fdcan_filter.FilterID1 = 0x00;
        fdcan_filter.FilterID2 = 0x00;
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_DeInit(hfdcan);
        HAL_FDCAN_Init(hfdcan);
        HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(hfdcan);
    }
}
//BUS OFF
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 检查是否为 BUSOFF 状态
    if(ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        error_code = can_busy;
        check = 1;
        // 退出受限模式（自动重启）
        HAL_FDCAN_ExitRestrictedOperationMode(hfdcan);
        if(hfdcan == &hfdcan1 || hfdcan == &hfdcan3)
        {
            can_filter_init(hfdcan, 0);
        }
        else if(hfdcan == &hfdcan2)
        {
            can_filter_init(hfdcan, 1);
        }
        HAL_FDCAN_Start(hfdcan);
    }
}
