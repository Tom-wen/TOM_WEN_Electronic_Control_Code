
#include "stm32f10x.h"                  // Device header
#include "headfile.h"
#include "OLED.h"
#include "Encoder.h"
#include "LINE_FOLLOWER.h"
#include "TIMER.h"
#include "OPEN_MV.h"
//------------------------JY61P-----------------------------//
uint16_t RxData0,RxData1,RxData2;
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);



//-----------------------------------------------------//

uint8_t KeyNum;		//定义用于接收按键键码的变量
int16_t i=0;//定义待被旋转编码器调节的变量
int16_t key=0;
int16_t key_num=0;
int16_t speed=25;//25满电量状态
//修复相关问题

int main(void)
{
	SystemInit();
	//JY61P陀螺仪的初始化
	Usart2Init(9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	AutoScanSensor(); 
	/*模块初始化*/
	uart4_init(9600);
	line_follower_init();
	Timer_Init(10000-1,7200-1);
	Key_Init();			//按键初始化	
	Delay_ms(1000);
	while (1)
	{
		key=Key_GetNum();
		key_num=key_num+key;


		//Motor_MECNAMU_SetSpeed(x,y);
		//示例：前进（0,-10）左转（-10，0）
		//示例：后退（0, 10）右转（ 10，0）

		//以下为满电量状态参数
		if (key_num % 2==1)
		{
			Motor_MECNAMU_SetSpeed(-speed,0);//30   左
			Delay_ms(7400);

			//6500

			//第一个箱子
			Motor_MECNAMU_SetSpeed(0,-speed);//    前
			Delay_s(5);
			Motor_MECNAMU_SetSpeed(0,speed);//    后
			Delay_s(2);
			Motor_MECNAMU_SetSpeed(-speed,0);//  左
			Delay_ms(4500);

			// 5 2 4500

			//第二个箱子
			Motor_MECNAMU_SetSpeed(0,-speed);//    前
			Delay_ms(2000);
			Motor_MECNAMU_SetSpeed(0,speed);//    后
			Delay_ms(2000);
			Motor_MECNAMU_SetSpeed(-speed,0);//   左
			Delay_ms(4500);

			// 2 2 4500

			//第三个箱子
			Motor_MECNAMU_SetSpeed(0,-speed);//    前
			Delay_s(2);
			Motor_MECNAMU_SetSpeed(0,speed);//    后
			Delay_s(2);
			Motor_MECNAMU_SetSpeed(-speed,0);//  左
			Delay_s(2);
			Motor_MECNAMU_SetSpeed(0,0);
			
			//2 2 2
		}
		else
		{
			Motor_MECNAMU_SetSpeed(0,0);//30
		}				
	}

}


// 定时器中断服务函数 - 用于定期执行控制算法
void TIM4_IRQHandler(void) 
{           

}

void CopeCmdData(unsigned char ucData)
{
	 static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	 s_ucData[s_ucRxCnt++] = ucData;
	 if(s_ucRxCnt<3)return;										//Less than three data returned
	 if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	 if(s_ucRxCnt >= 3)
	 {
		 if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		 {
		  	s_cCmd = s_ucData[0];
			  memset(s_ucData,0,50);//
			  s_ucRxCnt = 0;
	   } 
		 else 
		 {
			 s_ucData[0] = s_ucData[1];
			 s_ucData[1] = s_ucData[2];
			 s_ucRxCnt = 2;
			}
	  }
}
void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart2Init(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delay_ms(100);
			if(s_cDataUpdate != 0)
			{
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
}

