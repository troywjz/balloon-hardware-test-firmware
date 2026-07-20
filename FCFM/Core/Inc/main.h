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
#define VALVE1_Pin GPIO_PIN_13
#define VALVE1_GPIO_Port GPIOC
#define VALVE2_Pin GPIO_PIN_14
#define VALVE2_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT2_Pin GPIO_PIN_1
#define IMU_INT2_GPIO_Port GPIOC
#define RADIO_RXEN_Pin GPIO_PIN_2
#define RADIO_RXEN_GPIO_Port GPIOC
#define RADIO_TXEN_Pin GPIO_PIN_3
#define RADIO_TXEN_GPIO_Port GPIOC
#define PUMP2_IN2_Pin GPIO_PIN_2
#define PUMP2_IN2_GPIO_Port GPIOA
#define PUMP2_IN1_Pin GPIO_PIN_3
#define PUMP2_IN1_GPIO_Port GPIOA
#define SPI1_CS_IMU_Pin GPIO_PIN_4
#define SPI1_CS_IMU_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_4
#define SDIO_CD_GPIO_Port GPIOC
#define PUMP1_IN1_Pin GPIO_PIN_2
#define PUMP1_IN1_GPIO_Port GPIOB
#define RADIO_RST_Pin GPIO_PIN_10
#define RADIO_RST_GPIO_Port GPIOB
#define RADIO_DIO1_Pin GPIO_PIN_11
#define RADIO_DIO1_GPIO_Port GPIOB
#define SPI2_CS_RADIO_Pin GPIO_PIN_12
#define SPI2_CS_RADIO_GPIO_Port GPIOB
#define RADIO_BUSY_Pin GPIO_PIN_8
#define RADIO_BUSY_GPIO_Port GPIOA
#define MOTOR_FAULT_Pin GPIO_PIN_10
#define MOTOR_FAULT_GPIO_Port GPIOA
#define MOTOR_SLEEP_Pin GPIO_PIN_15
#define MOTOR_SLEEP_GPIO_Port GPIOA
#define PUMP1_IN2_Pin GPIO_PIN_3
#define PUMP1_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
