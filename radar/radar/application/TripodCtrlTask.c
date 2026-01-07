#include "task_public.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "can.h"
#include "GQ_Motor.h"
#include "PID.h"
#include "USART_user.h"
#include "aRGB.h"
uint8_t rx_data[24] = {0};

// 外部变量声明
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
    gimbal_pitch_init();
    

    user_usart_init();

    
    while (1) 
		{
            gimbal_set_control();
			gimbal_PID_Calc(&gimbal_motor_t_yaw);
            gimbal_PID_Calc(&gimbal_motor_t_pitch);
			CAN_cmd_gimbal(&gimbal_motor_t_yaw);
            CAN_cmd_gimbal(&gimbal_motor_t_pitch);
            osDelay(1);
            
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
            // 检测ID，如果ID为4则写入yaw轴，如果ID为1则写入pitch轴

            if (rx_header.StdId == 1024) 
            {
                update(&gimbal_motor_t_yaw, rx_data, &rx_header);
            } 
            else if (rx_header.StdId == 256) 
            {
                update(&gimbal_motor_t_pitch, rx_data, &rx_header);
            }
        }
         
     }
 }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim13) {
//         //33.3Hz
//         tripod.Ctrl(&tripod, vision_x_pid.calc(&vision_x_pid, offset_x), 
//                            vision_y_pid.calc(&vision_y_pid, offset_y));
//     }
// }


void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  //零位复位
    rezero_pos(&hcan1,1);
    conf_write(&hcan1,1);
    rezero_pos(&hcan1,4);
    conf_write(&hcan1,4);
    aRGB_led_show(0xFFFF0000); // 红灯
  /* USER CODE END EXTI0_IRQn 1 */
}