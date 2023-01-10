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
#define LED_D13_Pin GPIO_PIN_5
#define LED_D13_GPIO_Port GPIOA
#define LED_D12_Pin GPIO_PIN_6
#define LED_D12_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Btn_D5_Pin GPIO_PIN_11
#define Btn_D5_GPIO_Port GPIOE
#define Btn_D5_EXTI_IRQn EXTI15_10_IRQn
#define Btn_D4_Pin GPIO_PIN_14
#define Btn_D4_GPIO_Port GPIOE
#define Btn_D4_EXTI_IRQn EXTI15_10_IRQn
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define LED_D10_Pin GPIO_PIN_14
#define LED_D10_GPIO_Port GPIOD
#define LED_D9_Pin GPIO_PIN_15
#define LED_D9_GPIO_Port GPIOD
#define Btn_D3_Pin GPIO_PIN_6
#define Btn_D3_GPIO_Port GPIOG
#define Btn_D6_Pin GPIO_PIN_8
#define Btn_D6_GPIO_Port GPIOA
#define LED_D8_Pin GPIO_PIN_9
#define LED_D8_GPIO_Port GPIOG
#define LED_D7_Pin GPIO_PIN_12
#define LED_D7_GPIO_Port GPIOG
#define LED_D11_Pin GPIO_PIN_5
#define LED_D11_GPIO_Port GPIOB
#define LED_D14_Pin GPIO_PIN_9
#define LED_D14_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
void   MX_ETH_Init(void);
void   MX_GPIO_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
