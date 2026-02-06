#include "task_public.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "can.h"
#include "GQ_Motor.h"
#include "PID.h"
#include "USART_user.h"
#include "aRGB.h"
#include "remote_control.h"
#include "math.h"
uint8_t rx_data[24] = {0};

// 外部全局变量
extern float INS_angle[3];       // yaw,pitch,roll



void TripodCtrlTask(void *argument) 
{
    
    user_usart_init();
    //Emm_V5_Restore_Motor(1,&hcan1);
    //Emm_V5_Restore_Motor(1,&hcan2);
    //Emm_V5_Pos_Control(1, 1, 20, 0, 200, 0, 0,&hcan1);
    //Emm_V5_Modify_Ctrl_Mode(1,true,1,&hcan1);
    //Emm_V5_Modify_Ctrl_Mode(1,true,1,&hcan2);
    //Emm_V5_Modify_PID_Params(1, true, 18000, 10, 18000,&hcan1);
    //Emm_V5_Modify_PID_Params(1, true, 18000, 10, 18000,&hcan2);
  //Emm_V5_Pos_Control(1, 1, 1, 1, 200, 0, 0,&hcan1);
  osDelay(10);



    while (1)
	{
    if (gimbal_motor_t_pitch.radar_add <0)
    {
    Emm_V5_Pos_Control(1, 1, 2, 1, (int)fabsf(gimbal_motor_t_pitch.radar_add), 0, 0,&hcan1);
    gimbal_motor_t_pitch.radar_add=0;
    }
    else
    {
    Emm_V5_Pos_Control(1, 0, 2, 1, (int)gimbal_motor_t_pitch.radar_add, 0, 0,&hcan1);
    gimbal_motor_t_pitch.radar_add=0;
    }
     osDelay(50);
    if (gimbal_motor_t_yaw.radar_add <0)
    {
    Emm_V5_Pos_Control(1, 1, 2, 1, (int)fabsf(gimbal_motor_t_yaw.radar_add), 0, 0,&hcan2);
    gimbal_motor_t_yaw.radar_add=0;
    }
    else
    {
    Emm_V5_Pos_Control(1, 0, 2, 1, (int)gimbal_motor_t_yaw.radar_add, 0, 0,&hcan2);
    gimbal_motor_t_yaw.radar_add=0;
    }
    //handle_remote_disconnect();//检测遥控器连接状态
    osDelay(50);
    // //VOFA_Send_Float_Data();
    }
}




void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  //校准校准
    aRGB_led_show(0xFFFF0000); //红色
  /* USER CODE END EXTI0_IRQn 1 */
}