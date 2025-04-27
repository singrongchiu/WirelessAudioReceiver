/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define FFT_N 1024
#define MESSAGE_LENGTH 1024
#define FREQ0INDEX1 64 // for 200kHz sampling rate, freq = 12500
#define FREQ0INDEX2 128 // harmonics for square wave
#define FREQ0INDEX3 192 //
#define FREQ0INDEX4 256 //
#define FREQ1INDEX1 160 // for 200kHz sampling rate, freq = 31250 (bin 153.6)
#define FREQ1INDEX2 320 //
#define FREQ1INDEX3 80 //
#define FREQ1INDEX4 480 //
#define RECEIVE0THRESHOLD 1500
#define RECEIVE0THRESHOLDTWO 1200
#define RECEIVE1THRESHOLD 1300
#define RECEIVE1THRESHOLDTWO 1200
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define LD1_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
