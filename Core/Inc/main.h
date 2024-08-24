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
#include "stm32f1xx_hal.h"

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
#define LORA_DI0_Pin GPIO_PIN_1
#define LORA_DI0_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_RST_Pin GPIO_PIN_1
#define LORA_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define PACKET_TRANSMIT_ERROR 0x01
#define BMP_INIT_ERROR 0x02
#define BMP_READ_ERROR 0x04
#define MPU_INIT_ERROR 0x08
#define GPS_NO_DATA 0x10
#define CO2_DATA_ERROR 0x20
#define MPU_READ_ERROR 0x40
#define SYSTEM_ERROR 0xFF

#define PACKET_LEN 59
#define CYCLE_TIME 1000

typedef struct {
	float lat;
	float lon;
	uint32_t secs;
} GPS;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
