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
#define BTN1_Pin GPIO_PIN_12
#define BTN1_GPIO_Port GPIOJ
#define CDET_Pin GPIO_PIN_13
#define CDET_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_13
#define BTN2_GPIO_Port GPIOJ
#define SEGA_Pin GPIO_PIN_0
#define SEGA_GPIO_Port GPIOJ
#define SEGDP_Pin GPIO_PIN_7
#define SEGDP_GPIO_Port GPIOJ
#define SEGG_Pin GPIO_PIN_6
#define SEGG_GPIO_Port GPIOJ
#define SEGB_Pin GPIO_PIN_1
#define SEGB_GPIO_Port GPIOJ
#define SEGF_Pin GPIO_PIN_5
#define SEGF_GPIO_Port GPIOJ
#define SEGC_Pin GPIO_PIN_2
#define SEGC_GPIO_Port GPIOJ
#define SEGD_Pin GPIO_PIN_3
#define SEGD_GPIO_Port GPIOJ
#define SEGE_Pin GPIO_PIN_4
#define SEGE_GPIO_Port GPIOJ

/* USER CODE BEGIN Private defines */

void showDigit(uint8_t digit);

void showNumber(uint32_t number, uint32_t delay);


void showLetter(char letter);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
