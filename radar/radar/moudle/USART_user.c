#include "USART_user.h"
#include "FreeRTOS.h"
#include "task.h"




uint16_t rx_len = 0;

uint8_t received_buffer[BUFFER_SIZE];
void user_usart_init(void)
{
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_IT(&huart6, received_buffer, BUFFER_SIZE);
}


void process_received_data(uint8_t *buf, uint16_t this_time_rx_len)
{

        if (buf[0] == 'V' && buf[1] == 'G' && buf[10] == 'V')
    {

        // 解析float数据 (协议为小端序)
        // yaw角度
        memcpy(&gimbal_motor_t_yaw.radar_add, &buf[2], sizeof(float));
        // pitch角度
        memcpy(&gimbal_motor_t_pitch.radar_add, &buf[6], sizeof(float));

    }

}

void USART6_IRQHandler(void)
{
    static uint16_t this_time_rx_len = 0;
    if (huart6.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        // 获取接收到的数据，清除RXNE标志位
        uint8_t received_data = huart6.Instance->DR;
        received_buffer[this_time_rx_len] = received_data;
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        this_time_rx_len++;
    }

    // IDLE中断，表示一帧数据接收完成
    else if (huart6.Instance->SR & UART_FLAG_IDLE)
    {
        
        huart6.Instance->DR;
        // IDLE标志清除方式
        __IO uint32_t temp_sr = huart6.Instance->SR;
        __IO uint8_t temp_dr = huart6.Instance->DR;
        
        rx_len = this_time_rx_len;
        
        // 处理接收到的数据
        if(this_time_rx_len > 0)
        {
            process_received_data(received_buffer, this_time_rx_len);
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart6, received_buffer, BUFFER_SIZE);
        this_time_rx_len = 0;
    }

}













//vofa+代码



/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
    USART1->DR = Byte;
    while (!(USART1->SR & UART_FLAG_TXE));
}
/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}
/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}



void Serial_SendFloat(float number) {
	uint8_t *pData = (uint8_t *)&number; // 取浮点数的二进制数据
	Serial_SendArray(pData, sizeof(float)); // 发送二进制格式的数据
}

void Serial_SendFloatArray(float *array, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        Serial_SendFloat(array[i]); // 发送单个浮点数的二进制数据
    }
}

/**
  * 函    数：VOFA+浮点数据发送函数
  * 参    数：ch1 - ch4 ：四个通道的浮点数据
  * 返 回 值：无
  * 说    明：按照VOFA+协议格式发送4个浮点数和帧尾标识
  */
void VOFA_Send_Float_Data(float ch1, float ch2, float ch3, float ch4)
{
    // 定义4个通道的浮点数据
    float fdata[4];
    // VOFA+协议帧尾标识
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    
    // 填充4个通道的数据
    fdata[0] = ch1;
    fdata[1] = ch2;
    fdata[2] = ch3;
    fdata[3] = ch4;
    
    // 使用VOFA+协议发送数据
    Serial_SendFloatArray(fdata, sizeof(fdata) / sizeof(fdata[0]));
    Serial_SendArray(tail, 4);
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/