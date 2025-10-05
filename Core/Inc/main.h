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
#include "stm32l0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRVOFF_Pin GPIO_PIN_14
#define DRVOFF_GPIO_Port GPIOC
#define NFAULT_Pin GPIO_PIN_15
#define NFAULT_GPIO_Port GPIOC
#define PWM_INLC_Pin GPIO_PIN_0
#define PWM_INLC_GPIO_Port GPIOA
#define PWM_INHA_Pin GPIO_PIN_1
#define PWM_INHA_GPIO_Port GPIOA
#define POCI_Pin GPIO_PIN_6
#define POCI_GPIO_Port GPIOA
#define PICO_Pin GPIO_PIN_7
#define PICO_GPIO_Port GPIOA
#define PWM_INHB_Pin GPIO_PIN_8
#define PWM_INHB_GPIO_Port GPIOA
#define PWM_INLB_Pin GPIO_PIN_9
#define PWM_INLB_GPIO_Port GPIOA
#define NSLEEP_Pin GPIO_PIN_12
#define NSLEEP_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define PWM_INHC_Pin GPIO_PIN_5
#define PWM_INHC_GPIO_Port GPIOB
#define PWM_INLCB6_Pin GPIO_PIN_6
#define PWM_INLCB6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
