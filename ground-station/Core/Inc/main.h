/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define RADIO_RXEN_Pin GPIO_PIN_2
#define RADIO_RXEN_GPIO_Port GPIOC
#define RADIO_TXEN_Pin GPIO_PIN_3
#define RADIO_TXEN_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_4
#define SDIO_CD_GPIO_Port GPIOC
#define RADIO_RST_Pin GPIO_PIN_10
#define RADIO_RST_GPIO_Port GPIOB
#define RADIO_DIO1_Pin GPIO_PIN_11
#define RADIO_DIO1_GPIO_Port GPIOB
#define RADIO_DIO1_EXTI_IRQn EXTI15_10_IRQn
#define RADIO_CS_Pin GPIO_PIN_12
#define RADIO_CS_GPIO_Port GPIOB
#define USB_DET_Pin GPIO_PIN_6
#define USB_DET_GPIO_Port GPIOC
#define RADIO_BUSY_Pin GPIO_PIN_8
#define RADIO_BUSY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
