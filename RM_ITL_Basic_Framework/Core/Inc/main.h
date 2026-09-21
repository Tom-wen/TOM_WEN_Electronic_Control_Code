/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_5V_Pin GPIO_PIN_15
#define Power_5V_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_INT2_Pin GPIO_PIN_14
#define SPI1_INT2_GPIO_Port GPIOE
#define SPI1_INT2_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_CS1_Pin GPIO_PIN_15
#define SPI1_CS1_GPIO_Port GPIOE
#define SPI1_CS2_Pin GPIO_PIN_10
#define SPI1_CS2_GPIO_Port GPIOB
#define SPI1_CS3_Pin GPIO_PIN_11
#define SPI1_CS3_GPIO_Port GPIOB
#define SPI1_INT1_Pin GPIO_PIN_10
#define SPI1_INT1_GPIO_Port GPIOD
#define SPI1_INT1_EXTI_IRQn EXTI15_10_IRQn
#define USERLED_Pin GPIO_PIN_10
#define USERLED_GPIO_Port GPIOC
#define SPI1_INT3_Pin GPIO_PIN_11
#define SPI1_INT3_GPIO_Port GPIOC
#define SPI1_INT3_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
