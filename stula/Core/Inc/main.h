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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define segG_Pin GPIO_PIN_0
#define segG_GPIO_Port GPIOC
#define segD_Pin GPIO_PIN_1
#define segD_GPIO_Port GPIOC
#define segE_Pin GPIO_PIN_2
#define segE_GPIO_Port GPIOC
#define segC_Pin GPIO_PIN_3
#define segC_GPIO_Port GPIOC
#define PV_Pin GPIO_PIN_1
#define PV_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define segB_Pin GPIO_PIN_4
#define segB_GPIO_Port GPIOC
#define segF_Pin GPIO_PIN_5
#define segF_GPIO_Port GPIOC
#define Button1_Pin GPIO_PIN_0
#define Button1_GPIO_Port GPIOB
#define Button1_EXTI_IRQn EXTI0_IRQn
#define segA_Pin GPIO_PIN_6
#define segA_GPIO_Port GPIOC
#define segDP_Pin GPIO_PIN_7
#define segDP_GPIO_Port GPIOC
#define COM3_Pin GPIO_PIN_8
#define COM3_GPIO_Port GPIOC
#define COM2_Pin GPIO_PIN_9
#define COM2_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define COM1_Pin GPIO_PIN_10
#define COM1_GPIO_Port GPIOC
#define COM4_Pin GPIO_PIN_11
#define COM4_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Button2_Pin GPIO_PIN_5
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
