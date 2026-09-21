#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "stm32h7xx.h"
#include "spi.h"
#include "tim.h"
#define WS2812_SPI_UNIT     hspi6

#define WS_H           168   // 1 码相对计数值
#define WS_L           84   // 0 码相对计数值
#define WS_REST        40   // 复位信号脉冲数量
#define LED_NUM         9   // WS2812灯个数
#define DATA_LEN       24   // WS2812数据长度，单个需要24个字节
#define WS2812_RST_NUM 50   // 官方复位时间为50us（40个周期），保险起见使用50个周期

extern SPI_HandleTypeDef WS2812_SPI_UNIT;
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void LED_Normal();
void LED_Warning();
void WS2812_Init(void);
void WS2812_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B);

#endif
