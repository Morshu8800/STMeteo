/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define up_Pin GPIO_PIN_9
#define up_GPIO_Port GPIOB
#define RES_Pin GPIO_PIN_1
#define RES_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_3
#define DC_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define LCD_VDD_Pin GPIO_PIN_7
#define LCD_VDD_GPIO_Port GPIOA
#define METEO_VDD_Pin GPIO_PIN_0
#define METEO_VDD_GPIO_Port GPIOB
#define MEMORY_VDD_Pin GPIO_PIN_1
#define MEMORY_VDD_GPIO_Port GPIOB
#define ALARM_INT_Pin GPIO_PIN_2
#define ALARM_INT_GPIO_Port GPIOB
#define ALARM_INT_EXTI_IRQn EXTI2_3_IRQn
#define backlight_Pin GPIO_PIN_8
#define backlight_GPIO_Port GPIOA
#define speaker_Pin GPIO_PIN_6
#define speaker_GPIO_Port GPIOC
#define alarm_disable_Pin GPIO_PIN_3
#define alarm_disable_GPIO_Port GPIOB
#define ok_Pin GPIO_PIN_7
#define ok_GPIO_Port GPIOB
#define ok_EXTI_IRQn EXTI4_15_IRQn
#define down_Pin GPIO_PIN_8
#define down_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
