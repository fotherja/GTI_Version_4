/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define DFSDM_Clk_Pin GPIO_PIN_2
#define DFSDM_Clk_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define Phase_BN_Pin GPIO_PIN_7
#define Phase_BN_GPIO_Port GPIOA
#define Phase_AN_Pin GPIO_PIN_0
#define Phase_AN_GPIO_Port GPIOB
#define Vbus_DIN_Pin GPIO_PIN_1
#define Vbus_DIN_GPIO_Port GPIOB
#define Igrid_DIN_Pin GPIO_PIN_12
#define Igrid_DIN_GPIO_Port GPIOB
#define Vgrid_DIN_Pin GPIO_PIN_14
#define Vgrid_DIN_GPIO_Port GPIOB
#define Icap_DIN_Pin GPIO_PIN_7
#define Icap_DIN_GPIO_Port GPIOC
#define Phase_B_Pin GPIO_PIN_8
#define Phase_B_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_9
#define Phase_A_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_11
#define Relay_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
