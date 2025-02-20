/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define AIMU_Heat_Pin GPIO_PIN_5
#define AIMU_Heat_GPIO_Port GPIOB
#define Debug_Pin GPIO_PIN_14
#define Debug_GPIO_Port GPIOG
#define RM_OLED_Pin GPIO_PIN_4
#define RM_OLED_GPIO_Port GPIOB
#define RM_OLEDB3_Pin GPIO_PIN_3
#define RM_OLEDB3_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define DR16_Pin GPIO_PIN_7
#define DR16_GPIO_Port GPIOB
#define DebugG9_Pin GPIO_PIN_9
#define DebugG9_GPIO_Port GPIOG
#define DR16A9_Pin GPIO_PIN_9
#define DR16A9_GPIO_Port GPIOA
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOG
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOG
#define A_IMU_Pin GPIO_PIN_7
#define A_IMU_GPIO_Port GPIOF
#define AIMU_HeatF6_Pin GPIO_PIN_6
#define AIMU_HeatF6_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOG
#define A_IMUF9_Pin GPIO_PIN_9
#define A_IMUF9_GPIO_Port GPIOF
#define A_IMUF8_Pin GPIO_PIN_8
#define A_IMUF8_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOH
#define RM_OLEDA6_Pin GPIO_PIN_6
#define RM_OLEDA6_GPIO_Port GPIOA
#define RM_Referee_Pin GPIO_PIN_9
#define RM_Referee_GPIO_Port GPIOD
#define RM_RefereeD8_Pin GPIO_PIN_8
#define RM_RefereeD8_GPIO_Port GPIOD
#define RM_OLEDA7_Pin GPIO_PIN_7
#define RM_OLEDA7_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
