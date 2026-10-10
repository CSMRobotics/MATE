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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOC
#define TempSense_Pin GPIO_PIN_0
#define TempSense_GPIO_Port GPIOC
#define VoltageSense_Pin GPIO_PIN_2
#define VoltageSense_GPIO_Port GPIOC
#define CurrentSense_Pin GPIO_PIN_3
#define CurrentSense_GPIO_Port GPIOC
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define BMI323_Int_2_Pin GPIO_PIN_4
#define BMI323_Int_2_GPIO_Port GPIOC
#define BMI323_Int_1_Pin GPIO_PIN_5
#define BMI323_Int_1_GPIO_Port GPIOC
#define Dshot7_Pin GPIO_PIN_1
#define Dshot7_GPIO_Port GPIOB
#define SPI1_CS2_Pin GPIO_PIN_2
#define SPI1_CS2_GPIO_Port GPIOB
#define Dshot8_Pin GPIO_PIN_10
#define Dshot8_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_14
#define PWM6_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_15
#define PWM5_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_8
#define PWM4_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_9
#define PWM3_GPIO_Port GPIOA
#define BNO086_Interrupt_Pin GPIO_PIN_10
#define BNO086_Interrupt_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_11
#define PWM1_GPIO_Port GPIOA
#define BNO086_WAKE_Pin GPIO_PIN_12
#define BNO086_WAKE_GPIO_Port GPIOA
#define Dshot6_Pin GPIO_PIN_15
#define Dshot6_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define Dshot5_Pin GPIO_PIN_4
#define Dshot5_GPIO_Port GPIOB
#define SPI3_CS2_Pin GPIO_PIN_5
#define SPI3_CS2_GPIO_Port GPIOB
#define Dshot4_Pin GPIO_PIN_6
#define Dshot4_GPIO_Port GPIOB
#define Dshot3_Pin GPIO_PIN_7
#define Dshot3_GPIO_Port GPIOB
#define Dshot2_Pin GPIO_PIN_8
#define Dshot2_GPIO_Port GPIOB
#define Dshot1_Pin GPIO_PIN_9
#define Dshot1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
