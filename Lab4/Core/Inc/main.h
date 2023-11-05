/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define BUTT_Pin GPIO_PIN_13
#define BUTT_GPIO_Port GPIOC
#define BUTT_EXTI_IRQn EXTI15_10_IRQn
#define PresSens_Pin GPIO_PIN_10
#define PresSens_GPIO_Port GPIOD
#define PresSens_EXTI_IRQn EXTI15_10_IRQn
#define GyroSens_Pin GPIO_PIN_11
#define GyroSens_GPIO_Port GPIOD
#define GyroSens_EXTI_IRQn EXTI15_10_IRQn
#define TempSens_Pin GPIO_PIN_15
#define TempSens_GPIO_Port GPIOD
#define TempSens_EXTI_IRQn EXTI15_10_IRQn
#define MagnetSens_Pin GPIO_PIN_8
#define MagnetSens_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
