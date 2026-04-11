bsp_usart
使用串口DMA进行接收与发送，可以减少CPU的浪费

使用方法：
1，如果要重新定义串口DMA发送，请定义一个发送完成标志位，使得发送不会堵塞
2，如果要重新定义串口DMA接收，请定义一个接收数组，用来接收串口数据
3，如果需要把数据进行处理，请定义一个串口回调函数，把数据放到模块层进行解析
如：//USART5
void bsp_usart5_set_callback(usart_rx_callback_t callback)
{
    usart5_user_callback = callback;
}
串口5接收遥控器数据，在remote_control里进行里数据解析
4，vofa+进行数据打印：
如果要进行小数发送请用void usart_vofa_send(UART_HandleTypeDef *huart)
这里使用的是vofa+的justfloat协议进行发送，justfloat可以发送小数。不用printf的原因是ARM-GCC编译器对C库进行了阉割，无法进行浮点数的支持，
如果需要使用printf发送小数的话，需要修改工具链文件，因此不如直接换一个协议发送，还能节省Flash和RAM,但这种发送速度好像更慢些，因为小数发送
是通过多个字节进行的

如果不发送小数可以用void usart_vofa_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
同时它还可以输出字符串，如usart_vofa_printf(&huart1,"helloworld");

增加了遥控器状态检测，需要使用到RTOS的计时器，如果不用RTOS可以把它注释了
//遥控器状态检测，如果不需要使用可以注释
#include "FreeRTOS.h"
#include "task.h"

//遥控器状态检测
volatile uint8_t sbus_online = 0;
volatile uint32_t last_sbus_recv_time = 0;  // 最后接收时间

错误回调函数：
使得DMA串口接收时打断点后重启依然能接收数据