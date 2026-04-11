#include "bsp_usart.h"
//遥控器状态检测，如果不需要使用可以注释
#include "FreeRTOS.h"
#include "task.h"
#include "PID_Test.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "Circular_buffer.h"

PowerMonitor_Data_t power_data;

//遥控器状态检测
volatile uint8_t sbus_online = 0;//1是在线，0是离线
volatile uint32_t last_sbus_recv_time = 0;  // 最后接收时间

//vofa发送需要（justfloat协议）
#define MAX_CHANNEL 10

#define BYTE0(dwTemp)       (*(char *)(dwTemp))
#define BYTE1(dwTemp)       (*((char *)(dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(dwTemp) + 3))
float *UserData[MAX_CHANNEL]={0};//only transmit float
unsigned char Data_Number = 0;
unsigned char data_to_send[4*MAX_CHANNEL+4] = {0};

// 建议每个串口各自有一个发送完成标志
static volatile uint8_t usart1_tx_dma_done = 1;
static volatile uint8_t usart2_tx_dma_done = 1;
static volatile uint8_t usart3_tx_dma_done = 1;
static volatile uint8_t usart7_tx_dma_done = 1;
static volatile uint8_t usart10_tx_dma_done = 1;
// ... 你有几个串口就定义几个

// 为每个CAN口声明独立的回调函数指针
static usart_rx_callback_t usart5_user_callback = NULL;//usart5
static usart_rx_callback_t usart7_user_callback = NULL;//usart7

uint8_t usart1_rx_buf[256];//串口1接收数据
uint8_t usart2_rx_buf[256];//串口2接收数据
uint8_t usart7_rx_buf[256];//串口2接收数据
uint8_t sbus_buf[25];//串口5接收数据,只进行遥控器接收，不发送
uint8_t usart10_rx_buf[256];//串口10接收数据，裁判系统常规链路数据

/* 先定义好存放数据的空间，队列大小是5，这里就定义5个。 */
UART_RX_TypeDef uart3_rx_data_t[UART_BUFFER_QUANTITY];

/* 定义一个变量用作分配上面定义的数据 */
uint8_t uart3_buff_ctrl = 0;

/* 引用串口DMA handle  */
extern DMA_HandleTypeDef hdma_usart3_rx;

/* 队列 handle。去freertos.c文件里面找CubeMX给你创建好的。 */
extern osMessageQueueId_t rs485_queueHandle;

//...串口...同理

// 为每个USART口设置独立的回调函数
//USART5
void bsp_usart5_set_callback(usart_rx_callback_t callback)
{
    usart5_user_callback = callback;
}
//USART7
void bsp_usart7_set_callback(usart_rx_callback_t callback)
{
    usart7_user_callback = callback;
}
/**
 * @brief  通用USART DMA发送函数，支持多串口
 * @param  huart  要发送的串口句柄指针（如 &huart1）
 * @param  data   发送数据缓冲区指针
 * @param  len    发送数据长度
 * @retval 1 启动成功，0 启动失败或上一次还未发送完成
 */
uint8_t usart_tx_dma_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    volatile uint8_t *tx_done = NULL;
    if(huart->Instance == USART1)
        tx_done = &usart1_tx_dma_done;
    else if(huart->Instance == USART2)
        tx_done = &usart2_tx_dma_done;
    else if(huart->Instance == USART3)
        tx_done = &usart3_tx_dma_done;        
    else if(huart->Instance == UART7)
        tx_done = &usart7_tx_dma_done;    
    else if(huart->Instance == USART10)
        tx_done = &usart10_tx_dma_done;
    // ... 其它串口同理

    if(tx_done && *tx_done)
    {
        *tx_done = 0;
        if(HAL_UART_Transmit_DMA(huart, data, len) == HAL_OK)
            return 1; // 启动成功
        else
        {
            *tx_done = 1; // 启动失败，恢复标志
            return 0;
        }
    }
    return 0; // 上一次还没发完
}

/**
 * @brief  串口DMA发送完成中断回调函数（由HAL库自动调用）
 * @param  huart  触发回调的串口句柄指针
 * @note   该函数在每次DMA发送完成后自动被HAL库调用，用于设置发送完成标志，
 *         以便允许下一次DMA发送。支持多串口。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
        usart1_tx_dma_done = 1;
    else if(huart->Instance == USART2)
        usart2_tx_dma_done = 1;
    else if(huart->Instance == USART3)
        usart3_tx_dma_done = 1;        
    else if(huart->Instance == UART7)
        usart7_tx_dma_done = 1;   
    else if(huart->Instance == USART10)
        usart10_tx_dma_done = 1;   
    // ... 其它串口同理
}

/**
 * @brief  启动串口DMA接收（带空闲中断，适合变长数据）
 * @param  huart  串口句柄指针（如 &huart1）
 * @param  buf    接收缓冲区指针
 * @param  len    接收缓冲区长度
 * @retval 1 启动成功，0 启动失败
 * @note   使用 HAL_UARTEx_ReceiveToIdle_DMA 启动DMA接收，接收完成或遇到空闲中断时会自动触发回调。
 */
uint8_t usart_rx_dma_start(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
    HAL_UART_AbortReceive(huart);
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, len) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);//禁止半满中断
        return 1;
    }
    else
    {
        return 0;
    }

}

//错误处理回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{  
    if(huart == &huart5)
    {
        // 清空缓冲，置位一些标志，具体看函数内部
        HAL_UART_AbortReceive(&huart5);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_buf, sizeof(sbus_buf));
    }
    else if(huart == &huart1)
    {
        HAL_UART_AbortReceive(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));        
    }
    else if(huart == &huart2)
    {
        HAL_UART_AbortReceive(&huart2);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));
    }
    else if(huart == &huart3)
    {
        HAL_UART_AbortReceive(&huart3);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART_BUFFER_SIZE);
    }    
    else if(huart == &huart7)
    {
        HAL_UART_AbortReceive(&huart7);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7_rx_buf, sizeof(usart7_rx_buf));
    }
    else if(huart == &huart10)
    {
        HAL_UART_AbortReceive(&huart10);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_rx_buf, sizeof(usart10_rx_buf));
    }
}

/**
 * @brief  串口DMA接收完成（空闲中断）回调函数（由HAL库自动调用）
 * @param  huart  触发回调的串口句柄指针
 * @param  Size   实际接收到的数据长度
 * @note   该函数在每次DMA接收遇到空闲中断或接收满时自动被HAL库调用。
 *         在这里可以处理接收到的数据，并重新启动DMA接收，保证持续接收。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 用户自定义数据处理
    extern void usart_rx_data_process(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);
    if(huart->Instance == USART1)
    {
        usart_rx_dma_start(huart, usart1_rx_buf, sizeof(usart1_rx_buf));
    }
    else if(huart->Instance == USART2)
    {
        power_data_clac((char *)usart2_rx_buf, &power_data);
        usart_rx_dma_start(huart, usart2_rx_buf, sizeof(usart2_rx_buf));
    }
    else if(huart->Instance == UART5)
    {
        // 更新接收时间戳
        #ifdef DJI_REMOTE
            last_sbus_recv_time = xTaskGetTickCount();//如果不使用RTOS可以注释
            // 标记在线
            sbus_online = 1;
        #else
            if((sbus_buf[23] >> 2) & 0x01)
            {
                sbus_online = 0;
            }
            else 
            {
                sbus_online = 1;
            }
        #endif

        // 处理遥控器数据
        if(usart5_user_callback != NULL)
        {
            usart5_user_callback(sbus_buf);
        }
        usart_rx_dma_start(huart, sbus_buf, sizeof(sbus_buf));
    }
    else if(huart->Instance == UART7)
    {
        // 更新接收时间戳
        #ifdef VT_03_REMOTE
            last_sbus_recv_time = xTaskGetTickCount();//如果不使用RTOS可以注释
            // 标记在线
            sbus_online = 1;
        #endif

        // 处理遥控器数据
        if(usart7_user_callback != NULL)
        {
            usart7_user_callback(usart7_rx_buf);
        }
        usart_rx_dma_start(&huart7, usart7_rx_buf, sizeof(usart7_rx_buf));  // USART7 DMA接收初始化        
    }
    // ... 其它串口同理
    else if(huart->Instance == USART10) // 判断是否是空闲中断
    {
        Command_Write(usart10_rx_buf, Size);
        /* 重启开始DMA传输 */
        usart_rx_dma_start(&huart10, usart10_rx_buf, sizeof(usart10_rx_buf));  // USART10 DMA接收初始化
    }
    else if(huart->Instance == USART3) // 判断是否是空闲中断
    {
        UART_RX_TypeDef *pUartData1; /* 定义指向创建串口数据的指针 */     
        
        /* 计算接收到的数据长度，放进串口数据结构体中 */
        uart3_rx_data_t[uart3_buff_ctrl].size = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        /* 将这个串口数据的结构体地址给指针 */
        pUartData1 = &uart3_rx_data_t[uart3_buff_ctrl];
        /* 把指向串口接收数据的指针放入消息队列。注意，这里传的是指针的地址，也就是指针串口数据结构体的指针的地址不能传pUartData本身 */
        if(rs485_queueHandle != NULL)//避免出现串口进入接收中断，但是消息队列还没初始化的问题
        {
            xQueueSendFromISR(rs485_queueHandle, &pUartData1, NULL);
        }
        /* 这里进行加一，使其下一次DMA传输时将接收的数据放到下一个串口缓冲区中 */
        uart3_buff_ctrl++;
        /* 取余操作防止越界 */
        uart3_buff_ctrl %= UART_BUFFER_QUANTITY;
        /* 重启开始DMA传输 */
        usart_rx_dma_start(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART_BUFFER_SIZE);  // USART3 DMA接收初始化
    }
}

/**
 * @brief  用户自定义的串口DMA接收数据处理函数（弱定义，可被用户重写）
 * @param  huart  当前接收数据的串口句柄指针
 * @param  buf    接收到的数据缓冲区指针
 * @param  len    实际接收到的数据长度
 * @note   用户可在此函数中实现协议解析、数据拷贝、消息通知等功能。
 *         该函数为弱定义（__weak），用户可在其它文件中重新实现以覆盖默认实现。
 *         下面的 HAL_UART_Transmit_DMA(huart, buf, len) 是回环测试用的，可以删除。
 */
__weak void usart_rx_data_process(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
    // 用户自己实现，比如解析协议、拷贝数据等
    // 例：printf("收到%d字节\r\n", len);
    HAL_UART_Transmit_DMA(huart, buf, len);//自己加了一个回传用来测试，可以删除
}

/**
 * @brief  通用串口DMA printf函数，支持任意串口
 * @param  huart  串口句柄指针（如 &huart1）
 * @param  fmt    格式化字符串
 * @param  ...    可变参数
 */
void usart_vofa_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    static uint8_t tx_buf[256]; // 发送缓冲区
    va_list ap;
    uint16_t len;

    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap); // 格式化字符串到缓冲区
    va_end(ap);

    // 防止溢出
    if(len > sizeof(tx_buf)) len = sizeof(tx_buf);

    // 用你的DMA发送函数，自动防止重复启动
    usart_tx_dma_send(huart, tx_buf, len);
}

//vofa+justfloat协议进行串口输出，可以输出小数
void Upper_Computer_Init(float* addr)
{
    if(Data_Number < MAX_CHANNEL)UserData[Data_Number++]=addr;
}

void usart_vofa_send(UART_HandleTypeDef *huart)
{
	unsigned char cnt = 0;

	for(int i=0;i<Data_Number;i++)
	{
		data_to_send[cnt++] = BYTE0(UserData[i]);
		data_to_send[cnt++] = BYTE1(UserData[i]);
		data_to_send[cnt++] = BYTE2(UserData[i]);
		data_to_send[cnt++] = BYTE3(UserData[i]);
	}

	data_to_send[cnt++] = 0x00;
	data_to_send[cnt++] = 0x00;
	data_to_send[cnt++] = 0x80;
	data_to_send[cnt++] = 0x7F;

	while((huart)->gState != HAL_UART_STATE_READY); //如果上一次包没有发送完成，会导致这个包被丢弃
	usart_tx_dma_send(huart, data_to_send, cnt);
}


// 解析函数
void power_data_clac(uint8_t *buf, PowerMonitor_Data_t *power_data)
{
    // 电压：假设格式为 XX.XX
    power_data->voltage = (buf[8] - '0') * 10 + (buf[9] - '0') 
                        + (buf[11] - '0') * 0.1 + (buf[12] - '0') * 0.01;

    // 电流：假设格式为 X.XX
    power_data->current = (buf[22] - '0') 
                        + (buf[24] - '0') * 0.1 + (buf[25] - '0') * 0.01;

    // 功率：根据是否有两位整数来调整
    if(buf[34] >= '0' && buf[34] <= '9')
    {
        // 格式：XX.XX (两位整数)
        power_data->power = (buf[33] - '0') * 10 + (buf[34] - '0') 
                          + (buf[36] - '0') * 0.1 + (buf[37] - '0') * 0.01;
    }
    else
    {
        // 格式：X.XX (一位整数)
        power_data->power = (buf[33] - '0') 
                          + (buf[35] - '0') * 0.1 + (buf[36] - '0') * 0.01;
    }
}

