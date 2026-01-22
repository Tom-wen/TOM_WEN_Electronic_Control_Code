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
uint8_t rx_data[24] = {0};

// 外部全局变量
extern float INS_angle[3];       // yaw,pitch,roll



// 函数声明
void CAN_InterfaceInit(void);


void TripodCtrlTask(void *argument) 
{
    // // 初始化CAN接口
    CAN_InterfaceInit();
    
    // // 初始化电机
    gimbal_set_control_init();
	gimbal_yaw_init();
    //osDelay(4);
    gimbal_pitch_init();
    //osDelay(1);
    

    user_usart_init();


    while (1)
	{

    gimbal_set_control();
	gimbal_yaw_PID_Calc(&gimbal_motor_t_yaw);
    gimbal_pitch_PID_Calc(&gimbal_motor_t_pitch);
    handle_remote_disconnect();//检测遥控器连接状态
	CAN_yaw_cmd_gimbal(&gimbal_motor_t_yaw);
    osDelay(1);
    CAN_pitch_cmd_gimbal(&gimbal_motor_t_pitch);
    osDelay(1);
	motor_read(&hcan1,4);
	osDelay(1);
    VOFA_Send_Float_Data(gimbal_motor_t_yaw.relative_angle_set,gimbal_motor_t_yaw.gimbal_motor_measure.motor_data.motor.position, gimbal_motor_t_pitch.relative_angle_set,gimbal_motor_t_pitch.relative_angle);


    }
}

void CAN_InterfaceInit(void) 
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
 {
     if (hcan == &hcan1)
     {
         CAN_RxHeaderTypeDef rx_header;
         HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);


        if (rx_header.DLC != 0)
        {
            // 根据ID判断ID为4的写入yaw轴，根据ID为1的写入pitch轴

            if (rx_header.StdId == 1024)
            {
               update_yaw(&gimbal_motor_t_yaw, rx_data, &rx_header);
            }
            else if (rx_header.StdId == 0x20A)   
            {
                get_motor_measure(&M6020_motor_measure, rx_data);
                update_pitch(&gimbal_motor_t_pitch, &M6020_motor_measure, &rx_header);
            }
        }

     }
 }




void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  //校准校准
    rezero_pos(&hcan1,1);
    osDelay(10);
    conf_write(&hcan1,1);
    osDelay(10);
    rezero_pos(&hcan1,4);
    osDelay(10);
    conf_write(&hcan1,4);
    aRGB_led_show(0xFFFF0000); //红色
  /* USER CODE END EXTI0_IRQn 1 */
}