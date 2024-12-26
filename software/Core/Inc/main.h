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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define BU1_Pin GPIO_PIN_14
#define BU1_GPIO_Port GPIOC
#define BU2_Pin GPIO_PIN_15
#define BU2_GPIO_Port GPIOC
#define BATT_sens_Pin GPIO_PIN_0
#define BATT_sens_GPIO_Port GPIOA
#define V_Pin GPIO_PIN_1
#define V_GPIO_Port GPIOA
#define R_Pin GPIO_PIN_2
#define R_GPIO_Port GPIOA
#define I_Pin GPIO_PIN_3
#define I_GPIO_Port GPIOA
#define charge_LC_Pin GPIO_PIN_4
#define charge_LC_GPIO_Port GPIOA
#define LC_input_Pin GPIO_PIN_5
#define LC_input_GPIO_Port GPIOA
#define L_on_Pin GPIO_PIN_6
#define L_on_GPIO_Port GPIOA
#define C_on_Pin GPIO_PIN_7
#define C_on_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_3
#define Relay_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_4
#define A_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_5
#define B_GPIO_Port GPIOB
#define C_Pin GPIO_PIN_6
#define C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
