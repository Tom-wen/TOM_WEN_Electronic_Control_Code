#include "bsp_can.h"
#include "detect.h"
#include "control_power.h"

uint8_t FreeLevel1 = 0;
uint8_t check = 0;

// 修改为数组，支持多个ID的数据存储
#define CAN_MAX_SUPPORTED_ID 11  // 最大支持的ID数量

CANRxData Rx_data1[CAN_MAX_SUPPORTED_ID];//CAN1接收数据
CANRxData Rx_data2[CAN_MAX_SUPPORTED_ID];//CAN2接收数据，支持多个ID
CANRxData Rx_data3[CAN_MAX_SUPPORTED_ID];//CAN2接收数据，支持多个ID

// 为每个CAN口声明独立的回调函数指针
static can_rx_callback_t can1_user_callback = NULL;//CAN1
static can_rx_callback_t can2_user_callback = NULL;//CAN2
static can_rx_callback_t can3_user_callback = NULL;//CAN3

// 为每个CAN口设置独立的回调函数
//CAN1
void bsp_can1_set_callback(can_rx_callback_t callback)
{
    can1_user_callback = callback;
}
//CAN2
void bsp_can2_set_callback(can_rx_callback_t callback)
{
    can2_user_callback = callback;
}
//CAN3
void bsp_can3_set_callback(can_rx_callback_t callback)
{
    can3_user_callback = callback;
}

//bsp_can初始化
void bsp_can_init(void)
{
	can_filter_init(&hfdcan1, 0);                             
    HAL_FDCAN_Start(&hfdcan1);
    can_filter_init(&hfdcan2, 0);                             
    HAL_FDCAN_Start(&hfdcan2);
    can_filter_init(&hfdcan3, 1);                             
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

	if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 0)
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
    
    // 从FIFO0中获取接收到的数据
    if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, rx_data) == HAL_OK)
    {
        // 查找该ID是否已经存在
        uint8_t index = 0;
        uint8_t found = 0;
        
        for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
            if(Rx_data1[i].id == pRxHeader.Identifier) {
                index = i;
                found = 1;
                break;
            }
        }

        // 如果未找到该ID，则分配一个新的位置
        if(!found) {
            // 寻找一个空闲的位置（id为0表示空闲）
            for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
                if(Rx_data1[i].id == 0) {
                    index = i;
                    Rx_data1[index].id = pRxHeader.Identifier;
                    break;
                }
            }
        }

        // 存储数据
        Rx_data1[index].id = pRxHeader.Identifier;
        Rx_data1[index].len = 8;
        memcpy(Rx_data1[index].data, rx_data, Rx_data1[index].len);

        // 如果设置了用户回调函数，则调用它来处理数据
        if(can1_user_callback != NULL)
        {
            can1_user_callback(&Rx_data1[index], CAN1);
        }
        
    }
}

//can2接收函数
void fdcan2_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t rx_data[8];
    
    // 从FIFO0中获取接收到的数据
    if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &pRxHeader, rx_data) == HAL_OK)
    {
        if(pRxHeader.Identifier == 0x212){
            Rx_data2[0].len = 4;
            memcpy(Rx_data2[0].data, rx_data, Rx_data2[0].len);
            // 如果设置了用户回调函数，则调用它来处理数据
            power_data_callback(&Rx_data2[0], CAN2);
        }
        else if(pRxHeader.Identifier == 0x0FF){
            Rx_data2[0].len = 8;
            memcpy(Rx_data2[0].data, rx_data, Rx_data2[0].len);
            super_cap_callback(&Rx_data2[0], CAN2);
        }
        else{
            // 查找该ID是否已经存在
            uint8_t index = 0;
            uint8_t found = 0;
            
            for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
                if(Rx_data2[i].id == pRxHeader.Identifier) {
                    index = i;
                    found = 1;
                    break;
                }
            }

            // 如果未找到该ID，则分配一个新的位置
            if(!found) {
                // 寻找一个空闲的位置（id为0表示空闲）
                for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
                    if(Rx_data2[i].id == 0) {
                        index = i;
                        Rx_data2[index].id = pRxHeader.Identifier;
                        break;
                    }
                }
            }

            // 存储数据
            Rx_data2[index].id = pRxHeader.Identifier;
            Rx_data2[index].len = 8;
            memcpy(Rx_data2[index].data, rx_data, Rx_data2[index].len);

            // 如果设置了用户回调函数，则调用它来处理数据
            if(can2_user_callback != NULL)
            {
                can2_user_callback(&Rx_data2[index], CAN2);
            }
        }
    }
}

//can3接收函数
void fdcan3_rx_callback(void)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t rx_data[8];
    
    // 从FIFO0中获取接收到的数据
    if(HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &pRxHeader, rx_data) == HAL_OK)
    {
        // 查找该ID是否已经存在
        uint8_t index = 0;
        uint8_t found = 0;
        
        for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
            if(Rx_data3[i].id == pRxHeader.Identifier) {
                index = i;
                found = 1;
                break;
            }
        }
        
        // 如果未找到该ID，则分配一个新的位置
        if(!found) {
            // 寻找一个空闲的位置（id为0表示空闲）
            for(uint8_t i = 0; i < CAN_MAX_SUPPORTED_ID; i++) {
                if(Rx_data3[i].id == 0) {
                    index = i;
                    Rx_data3[index].id = pRxHeader.Identifier;
                    break;
                }
            }
        }
        
        // 存储数据
        Rx_data3[index].id = pRxHeader.Identifier;
        Rx_data3[index].len = 8;
        memcpy(Rx_data3[index].data, rx_data, Rx_data3[index].len);
        
        // 如果设置了用户回调函数，则调用它来处理数据
        if(can3_user_callback != NULL)
        {
            can3_user_callback(&Rx_data3[index], CAN3);
        }
    }
}

//CANFIFO1中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
    else if(hfdcan == &hfdcan2)
    {
        fdcan2_rx_callback();
    }
}

//CANFIFO0中断回调函数
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan == &hfdcan3)
	{
		fdcan3_rx_callback();
	}
}

//CAN通信错误处理
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if(hfdcan == &hfdcan1 || hfdcan == &hfdcan2) 
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
    else if(hfdcan == &hfdcan3)
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
        if(hfdcan == &hfdcan1 || hfdcan == &hfdcan2)
        {
            can_filter_init(hfdcan, 0);
        }
        else if(hfdcan == &hfdcan3)
        {
            can_filter_init(hfdcan, 1);
        }
        HAL_FDCAN_Start(hfdcan);
    }
}


