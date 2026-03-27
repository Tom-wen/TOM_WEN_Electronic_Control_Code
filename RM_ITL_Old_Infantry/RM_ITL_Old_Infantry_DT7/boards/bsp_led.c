#include "bsp_led.h"

#define WS2812_LowLevel    0xC0     // 0�?
#define WS2812_HighLevel   0xF0     // 1�?

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

//正常工作时的LED亮绿�?
void LED_Normal()
{
    WS2812_Ctrl(0,10,0);
}

//出现问题时的LED亮红�?
void LED_Warning()
{
    WS2812_Ctrl(10,0,0);
}

void LED_RC_Disconnected()
{
    WS2812_Ctrl(0,0,10);
}

void LED_OFF()
{
    WS2812_Ctrl(0,0,0);
}
void LED_Warning_Motor()
{
    WS2812_Ctrl(10,10,10);
}