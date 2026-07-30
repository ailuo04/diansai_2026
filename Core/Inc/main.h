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
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOF
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOF
#define HWT101_SCL_Pin GPIO_PIN_0
#define HWT101_SCL_GPIO_Port GPIOC
#define HWT101_SDA_Pin GPIO_PIN_1
#define HWT101_SDA_GPIO_Port GPIOC
#define MOTOR_3A_Pin GPIO_PIN_13
#define MOTOR_3A_GPIO_Port GPIOF
#define MOTOR_3B_Pin GPIO_PIN_14
#define MOTOR_3B_GPIO_Port GPIOF
#define MOTOR_4A_Pin GPIO_PIN_15
#define MOTOR_4A_GPIO_Port GPIOF
#define MOTOR_4B_Pin GPIO_PIN_0
#define MOTOR_4B_GPIO_Port GPIOG
#define Key_Pin GPIO_PIN_1
#define Key_GPIO_Port GPIOG
#define Steering_1B_Pin GPIO_PIN_7
#define Steering_1B_GPIO_Port GPIOE
#define MOTOR_1A_Pin GPIO_PIN_4
#define MOTOR_1A_GPIO_Port GPIOG
#define MOTOR_2A_Pin GPIO_PIN_5
#define MOTOR_2A_GPIO_Port GPIOG
#define MOTOR_1B_Pin GPIO_PIN_6
#define MOTOR_1B_GPIO_Port GPIOG
#define MOTOR_2B_Pin GPIO_PIN_7
#define MOTOR_2B_GPIO_Port GPIOG
#define Steering_1A_Pin GPIO_PIN_0
#define Steering_1A_GPIO_Port GPIOD
#define Gray_1_Pin GPIO_PIN_6
#define Gray_1_GPIO_Port GPIOD
#define Gray_2_Pin GPIO_PIN_7
#define Gray_2_GPIO_Port GPIOD
#define Gray_3_Pin GPIO_PIN_10
#define Gray_3_GPIO_Port GPIOG
#define Gray_4_Pin GPIO_PIN_11
#define Gray_4_GPIO_Port GPIOG
#define Gray_5_Pin GPIO_PIN_15
#define Gray_5_GPIO_Port GPIOG
#define Gray_6_Pin GPIO_PIN_8
#define Gray_6_GPIO_Port GPIOB
#define Gray_7_Pin GPIO_PIN_9
#define Gray_7_GPIO_Port GPIOB
#define Gray_8_Pin GPIO_PIN_0
#define Gray_8_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
