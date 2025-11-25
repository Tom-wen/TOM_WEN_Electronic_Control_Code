#include "stm32f10x.h"                  // Device header
#include "headfile.h"
int ERR;
u8 flag_xunji=0;

void gray_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
}

unsigned char digtal(unsigned char channel)//1-8	  获取X通道数字值
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(gpio_get(GPIO_G, Pin_4) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(gpio_get(GPIO_G, Pin_5) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(gpio_get(GPIO_G, Pin_8) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(gpio_get(GPIO_G, Pin_9) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(gpio_get(GPIO_G, Pin_10) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(gpio_get(GPIO_G, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(gpio_get(GPIO_G, Pin_14) == 1) value = 1;
			else value = 0;  
			break;  
 		case 8: 
 			if(gpio_get(GPIO_G, Pin_15) == 1) value = 1;
 			else value = 0;  
 			break;   
	}
	return value; 
}

int read_sensor(void)
{
        if(D1==0&&D2==0&&D3==0&&D4==1&&D5==1&&D6==0&&D7==0&&D8==0)
        {
           ERR=0;
           flag_xunji=1;
        }
        if(D1==0&&D2==0&&D3==0&&D4==1&&D5==0&&D6==0&&D7==0&&D8==0)//偏右一点
        {
            ERR=-1;
            flag_xunji=1;
        }
        if(D1==0&&D2==0&&D3==1&&D4==1&&D5==0&&D6==0&&D7==0&&D8==0)//偏右多一点
        {
            ERR=-2;
            flag_xunji=1;
        }
        if(D1==0&&D2==0&&D3==1&&D4==0&&D5==0&&D6==0&&D7==0&&D8==0 )
        {
            ERR=-3;
            flag_xunji=1;
        }
				if(D1==0&&D2==1&&D3==1&&D4==0&&D5==0&&D6==0&&D7==0&&D8==0 )
        {
            ERR=-4;
            flag_xunji=1;
        }

        if(D1==0&&D2==1&&D3==0&&D4==0&&D5==0&&D6==0&&D7==0&&D8==0)
        {
            ERR=-5;
            flag_xunji=1;
        }
        if(D1==1&&D2==1&&D3==0&&D4==0&&D5==0&&D6==0&&D7==0&&D8==0)
        {
            ERR=-6;
            flag_xunji=1;
        }
        if(D1==1&&D2==0&&D3==0&&D4==0&&D5==0&&D6==0&&D7==0&&D8==0)
        {
            ERR=-7;
					  flag_xunji=1;
				}
        if(D1==0&&D2==0&&D3==0&&D4==0&&D5==1&&D6==0&&D7==0&&D8==0)
        {
            ERR=1;
					  flag_xunji=1;
				}
        if(D1==0&&D2==0&&D3==0&&D4==0&&D5==1&&D6==1&&D7==0&&D8==0)
        {
            ERR=2;
					  flag_xunji=1;
				}
        if(D1==0&&D2==0&&D3==0&&D4==0&&D5==0&&D6==1&&D7==0&&D8==0)
        {
            ERR=3;
					 flag_xunji=1;
				}
        if(D1==0&&D2==0&&D3==0&&D4==0&&D5==0&&D6==1&&D7==1&&D8==0)
        {
            ERR=4;
					  flag_xunji=1;
				}
        if(D1==0&&D2==0&&D3==0&&D4==0&&D5==0&&D6==0&&D7==1&&D8==0)
        {
            ERR=5;
					 flag_xunji=1;
				}
		    if(D1==0&&D2==0&&D3==0&&D4==0&&D5==0&&D6==0&&D7==1&&D8==1)
        {
            ERR=6;
					  flag_xunji=1;
				}
			  if(D1==0&&D2==0&&D3==0&&D4==0&&D5==0&&D6==0&&D7==0&&D8==1)
        {
            ERR=7;
					  flag_xunji=1;
				}
		return ERR;
}

// 新增函数：获取线路位置（用于PID输入）
float get_line_position(void)
{
    read_sensor();  // 更新传感器数据
    
    // 将ERR转换为连续的位置值
    // ERR范围：-7到7，转换为-3.5到3.5的浮点数
    return (float)ERR / 2.0f;
}

void Limit(int *motoA)
{
    if(*motoA > 800) *motoA = 800;
    if(*motoA < -800) *motoA = -800;
}


