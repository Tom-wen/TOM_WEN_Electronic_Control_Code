#include "task_public.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "can.h"
#include "GQ_Motor.h"
//#include "QD4310.h"
#include "PID.h"
//#include "Tripod.h"
uint8_t rx_data[24] = {0};

// 定义常量
static const float yaw_center = 0.28f;   // 云台偏航中心位置,单位: rad
static const float pitch_center = 0.32f; // 云台俯仰中心位置,单位: rad
static const float PI = 3.14159265359f;  // π常量

// 外部变量声明
extern float INS_angle[3];       // yaw,pitch,roll
extern float offset_x, offset_y; // 视觉偏移量

// // 电机实例
// QD4310 YawMotor;
// QD4310 PitchMotor;

// // 云台实例
// Tripod tripod;

// // PID控制器实例
// PID vision_x_pid;
// PID vision_y_pid;

// 函数声明
void CAN_InterfaceInit(void);

void TripodCtrlTask(void *argument) 
{
    // // 初始化CAN接口
    CAN_InterfaceInit();
    
    // // 初始化电机
	gimbal_yaw_init();
    // QD4310_init(&PitchMotor, &hcan1, 0x01);
    
    // // 使能电机
    // YawMotor.enable(&YawMotor);
    // PitchMotor.enable(&PitchMotor);
    
    // // 上电复位云台角度
    // YawMotor.setAngle(&YawMotor, yaw_center);
    // PitchMotor.setAngle(&PitchMotor, pitch_center);
    
    // // 初始化云台
    // PID position_pid = {POSITION_TYPE, 0.1f, 0.002f, 2.1f, 100, -100, 1, -1};
    // Tripod_init(&tripod, &YawMotor, &PitchMotor, yaw_center, pitch_center, &position_pid, 0.001f);
    
    // // 初始化视觉PID控制器
    // PID_init(&vision_x_pid, POSITION_TYPE, 60.0f, 0.0f, 30.0f, 10, -10, 10, -10);
    // PID_init(&vision_y_pid, POSITION_TYPE, -25.0f, 0.0f, -35.0f, 10, -10, 5, -5);
    
    // osDelay(pdMS_TO_TICKS(2000));
    // HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin, GPIO_PIN_SET); // 使能激光
    // HAL_TIM_Base_Start_IT(&htim13);                                   // 开启视觉闭环定时器
    // tripod.enable(&tripod);
    
    while (1) 
		{
            gimbal_set_contorl();
			gimbal_PID_Calc(&gimbal_motor_t_yaw);
			CAN_cmd_gimbal(&gimbal_motor_t_yaw);
            osDelay(1);
            
			
        // while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        // tripod.Ctrl_ISR(&tripod, INS_angle[0] / PI * 180);
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

 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
     if (hcan == &hcan1) 
     {
         CAN_RxHeaderTypeDef rx_header;
         HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        
           //  if (rx_header.StdId == 1024) 
			//				 {

                  if (rx_header.DLC != 0)
      {
                 update(&gimbal_motor_t_yaw,rx_data, &rx_header);
      }
    //           } 
//						 else if (rx_header.StdId == 0x01) 
//						{
//                 PitchMotor.update(&PitchMotor, rx_data);
//             }
         
     }
 }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim13) {
//         //33.3Hz
//         tripod.Ctrl(&tripod, vision_x_pid.calc(&vision_x_pid, offset_x), 
//                            vision_y_pid.calc(&vision_y_pid, offset_y));
//     }
// }