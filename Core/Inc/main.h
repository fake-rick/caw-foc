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
#define CKTIM 180000000
#define DEADTIME_NS 200
#define PWM_PRSC 0
#define PWM_FREQ 45000
#define PWM_PERIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))
#define DEADTIME CKTIM/1000000/2*DEADTIME_NS/1000
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define TIM1_BREAK_Pin GPIO_PIN_6
#define TIM1_BREAK_GPIO_Port GPIOC
#define DRV_CAL_Pin GPIO_PIN_7
#define DRV_CAL_GPIO_Port GPIOC
#define DRV_ENABLE_Pin GPIO_PIN_8
#define DRV_ENABLE_GPIO_Port GPIOC
#define DRV_FAULT_Pin GPIO_PIN_9
#define DRV_FAULT_GPIO_Port GPIOC
#define DRV_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define DRV_SPI_NSS_Pin GPIO_PIN_15
#define DRV_SPI_NSS_GPIO_Port GPIOA
#define DRV_SPI_SCK_Pin GPIO_PIN_10
#define DRV_SPI_SCK_GPIO_Port GPIOC
#define DRV_SPI_MISO_Pin GPIO_PIN_11
#define DRV_SPI_MISO_GPIO_Port GPIOC
#define DRV_SPI_MOSI_Pin GPIO_PIN_12
#define DRV_SPI_MOSI_GPIO_Port GPIOC
#define MCU_STA_Pin GPIO_PIN_2
#define MCU_STA_GPIO_Port GPIOD
#define DRV_STA_Pin GPIO_PIN_3
#define DRV_STA_GPIO_Port GPIOB
#define CAN_STB_Pin GPIO_PIN_4
#define CAN_STB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
