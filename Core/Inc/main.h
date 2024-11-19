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
#define USB_RESET_Pin GPIO_PIN_2
#define USB_RESET_GPIO_Port GPIOA
#define LED_OK_Pin GPIO_PIN_0
#define LED_OK_GPIO_Port GPIOG
#define LED_ERR_Pin GPIO_PIN_1
#define LED_ERR_GPIO_Port GPIOG
#define nCS_MAIN_SPI_MCU_Pin GPIO_PIN_0
#define nCS_MAIN_SPI_MCU_GPIO_Port GPIOD
#define CONV_MAIN_SPI_MCU_Pin GPIO_PIN_1
#define CONV_MAIN_SPI_MCU_GPIO_Port GPIOD
#define nCS_CAL_SPI_Pin GPIO_PIN_2
#define nCS_CAL_SPI_GPIO_Port GPIOD
#define CUR_POS_SEL_Pin GPIO_PIN_11
#define CUR_POS_SEL_GPIO_Port GPIOG
#define MEAS_POS_SEL_Pin GPIO_PIN_12
#define MEAS_POS_SEL_GPIO_Port GPIOG
#define MEAS_NEG_SEL_Pin GPIO_PIN_13
#define MEAS_NEG_SEL_GPIO_Port GPIOG
#define CUR_NEG_SEL_Pin GPIO_PIN_14
#define CUR_NEG_SEL_GPIO_Port GPIOG
#define GEN_FSYNC_MCU_Pin GPIO_PIN_4
#define GEN_FSYNC_MCU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
