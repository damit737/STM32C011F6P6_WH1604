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
#include "stm32c0xx_hal.h"

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
typedef struct{
	GPIO_TypeDef *Port;
	uint16_t Pin;
}sPhysicalButtonAttribute_t;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BL_PWM_Pin GPIO_PIN_7
#define BL_PWM_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_3
#define RS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define K_ESC_Pin GPIO_PIN_5
#define K_ESC_GPIO_Port GPIOA
#define K_RIGHT_Pin GPIO_PIN_6
#define K_RIGHT_GPIO_Port GPIOA
#define K_LEFT_Pin GPIO_PIN_7
#define K_LEFT_GPIO_Port GPIOA
#define K_DOWN_Pin GPIO_PIN_11
#define K_DOWN_GPIO_Port GPIOA
#define K_UP_Pin GPIO_PIN_12
#define K_UP_GPIO_Port GPIOA
#define K_OK_Pin GPIO_PIN_6
#define K_OK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
