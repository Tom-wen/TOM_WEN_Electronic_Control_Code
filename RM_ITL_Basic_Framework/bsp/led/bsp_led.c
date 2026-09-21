#include "bsp_led.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码

//LED点灯
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

//正常工作时的LED亮绿灯
void LED_Normal()
{
    WS2812_Ctrl(0,20,0);
}

//出现问题时的LED亮红灯
void LED_Warning()
{
    WS2812_Ctrl(20,0,0);
}

//显存数组，长度为 灯的数量*24+复位周期
uint16_t WS2812_RGB_Buff[LED_NUM*DATA_LEN+WS2812_RST_NUM] = {0}; 
 
/**
 * 函数：WS2812单灯设置函数
 * 参数：num:灯的位置，R、G、B分别为三个颜色通道的亮度，最大值为255
 * 作用：单独设置每一个WS2812的颜色
***/
void WS2812_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B)
{
  uint32_t indexx=(num*(3*8));
  for (uint8_t i = 0;i < 8;i++)
  {
	//填充数组
	WS2812_RGB_Buff[indexx+i]      = (G << i) & (0x80)?WS_H:WS_L;
	WS2812_RGB_Buff[indexx+i + 8]  = (R << i) & (0x80)?WS_H:WS_L;
	WS2812_RGB_Buff[indexx+i + 16] = (B << i) & (0x80)?WS_H:WS_L;
  }
}
 
//WS2812初始化函数
void WS2812_Init()
{
	//设置关闭所有灯
  for(int i=0;i<8;i++)
  {
	WS2812_Set(i,0,20,0);
  }
  //作用：调用DMA将显存中的内容实时搬运至定时器的比较寄存器
  HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1,(uint32_t *)WS2812_RGB_Buff,sizeof(WS2812_RGB_Buff)/sizeof(uint16_t)); 
}

