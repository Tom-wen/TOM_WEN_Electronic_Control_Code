/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis */
osThreadId_t chassisHandle;
uint32_t chassisBuffer[ 512 ];
osStaticThreadDef_t chassisControlBlock;
const osThreadAttr_t chassis_attributes = {
  .name = "chassis",
  .cb_mem = &chassisControlBlock,
  .cb_size = sizeof(chassisControlBlock),
  .stack_mem = &chassisBuffer[0],
  .stack_size = sizeof(chassisBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for detect */
osThreadId_t detectHandle;
uint32_t detectBuffer[ 512 ];
osStaticThreadDef_t detectControlBlock;
const osThreadAttr_t detect_attributes = {
  .name = "detect",
  .cb_mem = &detectControlBlock,
  .cb_size = sizeof(detectControlBlock),
  .stack_mem = &detectBuffer[0],
  .stack_size = sizeof(detectBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for INS */
osThreadId_t INSHandle;
uint32_t INSBuffer[ 1024 ];
osStaticThreadDef_t INSControlBlock;
const osThreadAttr_t INS_attributes = {
  .name = "INS",
  .cb_mem = &INSControlBlock,
  .cb_size = sizeof(INSControlBlock),
  .stack_mem = &INSBuffer[0],
  .stack_size = sizeof(INSBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbal */
osThreadId_t gimbalHandle;
uint32_t gimbalBuffer[ 512 ];
osStaticThreadDef_t gimbalControlBlock;
const osThreadAttr_t gimbal_attributes = {
  .name = "gimbal",
  .cb_mem = &gimbalControlBlock,
  .cb_size = sizeof(gimbalControlBlock),
  .stack_mem = &gimbalBuffer[0],
  .stack_size = sizeof(gimbalBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for shoot */
osThreadId_t shootHandle;
uint32_t shootBuffer[ 512 ];
osStaticThreadDef_t shootControlBlock;
const osThreadAttr_t shoot_attributes = {
  .name = "shoot",
  .cb_mem = &shootControlBlock,
  .cb_size = sizeof(shootControlBlock),
  .stack_mem = &shootBuffer[0],
  .stack_size = sizeof(shootBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hoisting */
osThreadId_t hoistingHandle;
uint32_t hoistingBuffer[ 512 ];
osStaticThreadDef_t hoistingControlBlock;
const osThreadAttr_t hoisting_attributes = {
  .name = "hoisting",
  .cb_mem = &hoistingControlBlock,
  .cb_size = sizeof(hoistingControlBlock),
  .stack_mem = &hoistingBuffer[0],
  .stack_size = sizeof(hoistingBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for referee */
osThreadId_t refereeHandle;
uint32_t refereeBuffer[ 512 ];
osStaticThreadDef_t refereeControlBlock;
const osThreadAttr_t referee_attributes = {
  .name = "referee",
  .cb_mem = &refereeControlBlock,
  .cb_size = sizeof(refereeControlBlock),
  .stack_mem = &refereeBuffer[0],
  .stack_size = sizeof(refereeBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void chassis_task(void *argument);
extern void detect_task(void *argument);
void INS_task(void *argument);
extern void gimbal_task(void *argument);
extern void shoot_task(void *argument);
extern void hoisting_task(void *argument);
extern void referee_usart_task(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chassis */
  chassisHandle = osThreadNew(chassis_task, NULL, &chassis_attributes);

  /* creation of detect */
  detectHandle = osThreadNew(detect_task, NULL, &detect_attributes);

  /* creation of INS */
  INSHandle = osThreadNew(INS_task, NULL, &INS_attributes);

  /* creation of gimbal */
  gimbalHandle = osThreadNew(gimbal_task, NULL, &gimbal_attributes);

  /* creation of shoot */
  shootHandle = osThreadNew(shoot_task, NULL, &shoot_attributes);

  /* creation of hoisting */
  hoistingHandle = osThreadNew(hoisting_task, NULL, &hoisting_attributes);

  /* creation of referee */
  refereeHandle = osThreadNew(referee_usart_task, NULL, &referee_attributes);

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

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the INS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void *argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

