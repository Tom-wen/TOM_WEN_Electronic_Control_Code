#include "headfile.h"

void gray_init()
{
	gpio_init(GPIO_B, Pin_12, IU);   // D1
	gpio_init(GPIO_B, Pin_13, IU);   // D2
	gpio_init(GPIO_B, Pin_14, IU);   // D3
	gpio_init(GPIO_B, Pin_15, IU);   // D4
	gpio_init(GPIO_A, Pin_8, IU);    // D5
	gpio_init(GPIO_A, Pin_12, IU);   // D6
	gpio_init(GPIO_C, Pin_14, IU);   // D7
	gpio_init(GPIO_C, Pin_15, IU);   // D8    
}

float g_cThisState = 0;//这次状态
float g_cLastState = 0;//上次状态
float g_fHD_PID_Out;//灰度传感器PID计算输出速度
float g_fHD_PID_Out1;//电机1的最后循迹PID控制速度
float g_fHD_PID_Out2;//电机2的最后循迹PID控制速度
//8765     4321
void GRAY_XUNJI(void)
{
			if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 1)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 0;//前运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 1)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 0;//前运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 1)&&(D5 == 1)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 0;//前运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 1)&&(D6 == 1)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = -0.5;//右边运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 1)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = -1;//右边运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 1)&&(D7 == 1)&&(D8 == 0))
			{
					g_cThisState = -1.5;
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 1)&&(D8 == 0))
			{
					g_cThisState = -2;//快速右转
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 1)&&(D8 == 1))
			{
					g_cThisState = -2.5;
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 1))
			{
					g_cThisState = -3;//更快速右转
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 1)&&(D4 == 1)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 0.5;//左边运动
			}
			else if((D1 == 0)&&(D2 == 0)&&(D3 == 1)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 1;//左边运动
			}
			else if((D1 == 0)&&(D2 == 1)&&(D3 == 1)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 1.5;
			}
			else if((D1 == 0)&&(D2 == 1)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 2;//快速左转
			}
			else if((D1 == 1)&&(D2 == 1)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 2.5;
			}
			else if((D1 == 1)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0)&&(D8 == 0))
			{
					g_cThisState = 3;//更快速左转
			}
			else if((D3==1)&&(D4==1)&&(D5==1)&&(D6==1))//0000 0000
		 {
				g_cThisState = 0;
     }
		 else if((D1==1)&&(D2==1)&&(D3==1)&&(D4==1)&&(D5==1)&&(D6==1)&&(D7==1)&&(D8==1))//0000 0000
		 {
				g_cThisState = 0;
     }
}

unsigned char digtal(unsigned char channel)//1-8	  获取X通道数字值
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(gpio_get(GPIO_B, Pin_12) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(gpio_get(GPIO_B, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(gpio_get(GPIO_B, Pin_14) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(gpio_get(GPIO_B, Pin_15) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(gpio_get(GPIO_A, Pin_8) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(gpio_get(GPIO_A, Pin_12) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(gpio_get(GPIO_C, Pin_14) == 1) value = 1;
			else value = 0;  
			break;  
 		case 8: 
 			if(gpio_get(GPIO_C, Pin_15) == 1) value = 1;
 			else value = 0;  
 			break;   
	}
	return value; 
}

