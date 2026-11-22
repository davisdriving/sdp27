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
#include "stm32g4xx_hal.h"

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
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_Pin GPIO_PIN_3
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_4
#define IN2_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_11
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_12
#define IN4_GPIO_Port GPIOB
#define BNO_RST_Pin GPIO_PIN_1
#define BNO_RST_GPIO_Port GPIOB
#define BNO_CS_Pin GPIO_PIN_2
#define BNO_CS_GPIO_Port GPIOB
#define SCL_085_Pin GPIO_PIN_13
#define SCL_085_GPIO_Port GPIOB
#define SDA_085_Pin GPIO_PIN_14
#define SDA_085_GPIO_Port GPIOB
#define DI_085_Pin GPIO_PIN_15
#define DI_085_GPIO_Port GPIOB
#define BNO_INT_Pin GPIO_PIN_10
#define BNO_INT_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_15
#define CAN_STB_GPIO_Port GPIOA
#define BOOT_SELECTION_Pin GPIO_PIN_5
#define BOOT_SELECTION_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
