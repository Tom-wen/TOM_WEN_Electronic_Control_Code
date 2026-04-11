/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Init.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_arm */
osThreadId_t app_armHandle;
uint32_t app_armBuffer[ 512 ];
osStaticThreadDef_t app_armControlBlock;
const osThreadAttr_t app_arm_attributes = {
  .name = "app_arm",
  .cb_mem = &app_armControlBlock,
  .cb_size = sizeof(app_armControlBlock),
  .stack_mem = &app_armBuffer[0],
  .stack_size = sizeof(app_armBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_ins */
osThreadId_t app_insHandle;
uint32_t app_insBuffer[ 1024 ];
osStaticThreadDef_t app_insControlBlock;
const osThreadAttr_t app_ins_attributes = {
  .name = "app_ins",
  .cb_mem = &app_insControlBlock,
  .cb_size = sizeof(app_insControlBlock),
  .stack_mem = &app_insBuffer[0],
  .stack_size = sizeof(app_insBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for app_chassis */
osThreadId_t app_chassisHandle;
uint32_t app_chassisBuffer[ 512 ];
osStaticThreadDef_t app_chassisControlBlock;
const osThreadAttr_t app_chassis_attributes = {
  .name = "app_chassis",
  .cb_mem = &app_chassisControlBlock,
  .cb_size = sizeof(app_chassisControlBlock),
  .stack_mem = &app_chassisBuffer[0],
  .stack_size = sizeof(app_chassisBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_detect */
osThreadId_t app_detectHandle;
uint32_t app_detectBuffer[ 512 ];
osStaticThreadDef_t app_detectControlBlock;
const osThreadAttr_t app_detect_attributes = {
  .name = "app_detect",
  .cb_mem = &app_detectControlBlock,
  .cb_size = sizeof(app_detectControlBlock),
  .stack_mem = &app_detectBuffer[0],
  .stack_size = sizeof(app_detectBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for app_gimbal */
osThreadId_t app_gimbalHandle;
uint32_t app_gimbalBuffer[ 512 ];
osStaticThreadDef_t app_gimbalControlBlock;
const osThreadAttr_t app_gimbal_attributes = {
  .name = "app_gimbal",
  .cb_mem = &app_gimbalControlBlock,
  .cb_size = sizeof(app_gimbalControlBlock),
  .stack_mem = &app_gimbalBuffer[0],
  .stack_size = sizeof(app_gimbalBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for app_data */
osThreadId_t app_dataHandle;
uint32_t app_dataBuffer[ 512 ];
osStaticThreadDef_t app_dataControlBlock;
const osThreadAttr_t app_data_attributes = {
  .name = "app_data",
  .cb_mem = &app_dataControlBlock,
  .cb_size = sizeof(app_dataControlBlock),
  .stack_mem = &app_dataBuffer[0],
  .stack_size = sizeof(app_dataBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_test */
osThreadId_t app_testHandle;
uint32_t app_testBuffer[ 256 ];
osStaticThreadDef_t app_testControlBlock;
const osThreadAttr_t app_test_attributes = {
  .name = "app_test",
  .cb_mem = &app_testControlBlock,
  .cb_size = sizeof(app_testControlBlock),
  .stack_mem = &app_testBuffer[0],
  .stack_size = sizeof(app_testBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_shoot */
osThreadId_t app_shootHandle;
uint32_t app_shootBuffer[ 512 ];
osStaticThreadDef_t app_shootControlBlock;
const osThreadAttr_t app_shoot_attributes = {
  .name = "app_shoot",
  .cb_mem = &app_shootControlBlock,
  .cb_size = sizeof(app_shootControlBlock),
  .stack_mem = &app_shootBuffer[0],
  .stack_size = sizeof(app_shootBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for app_referee */
osThreadId_t app_refereeHandle;
uint32_t app_refereeBuffer[ 1024 ];
osStaticThreadDef_t app_refereeControlBlock;
const osThreadAttr_t app_referee_attributes = {
  .name = "app_referee",
  .cb_mem = &app_refereeControlBlock,
  .cb_size = sizeof(app_refereeControlBlock),
  .stack_mem = &app_refereeBuffer[0],
  .stack_size = sizeof(app_refereeBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rs485_queue */
osMessageQueueId_t rs485_queueHandle;
uint8_t rs485_queueBuffer[ 5 * 4 ];
osStaticMessageQDef_t rs485_queueControlBlock;
const osMessageQueueAttr_t rs485_queue_attributes = {
  .name = "rs485_queue",
  .cb_mem = &rs485_queueControlBlock,
  .cb_size = sizeof(rs485_queueControlBlock),
  .mq_mem = &rs485_queueBuffer,
  .mq_size = sizeof(rs485_queueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern osMessageQueueId_t uart_queueHandle;
extern osMessageQueueId_t rs485_queueHandle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void arm_task(void *argument);
extern void ins_task(void *argument);
extern void chassis_task(void *argument);
extern void detect_task(void *argument);
extern void gimbal_task(void *argument);
extern void data_task(void *argument);
extern void test_task(void *argument);
extern void shoot_task(void *argument);
extern void referee_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rs485_queue */
  rs485_queueHandle = osMessageQueueNew (5, 4, &rs485_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of app_arm */
  app_armHandle = osThreadNew(arm_task, NULL, &app_arm_attributes);

  /* creation of app_ins */
  app_insHandle = osThreadNew(ins_task, NULL, &app_ins_attributes);

  /* creation of app_chassis */
  app_chassisHandle = osThreadNew(chassis_task, NULL, &app_chassis_attributes);

  /* creation of app_detect */
  app_detectHandle = osThreadNew(detect_task, NULL, &app_detect_attributes);

  /* creation of app_gimbal */
  app_gimbalHandle = osThreadNew(gimbal_task, NULL, &app_gimbal_attributes);

  /* creation of app_data */
  app_dataHandle = osThreadNew(data_task, NULL, &app_data_attributes);

  /* creation of app_test */
  app_testHandle = osThreadNew(test_task, NULL, &app_test_attributes);

  /* creation of app_shoot */
  app_shootHandle = osThreadNew(shoot_task, NULL, &app_shoot_attributes);

  /* creation of app_referee */
  app_refereeHandle = osThreadNew(referee_task, NULL, &app_referee_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

