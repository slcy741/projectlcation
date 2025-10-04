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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"  // 增加HAL库总头文件
#include "stm32f1xx_hal_rcc.h"    // RCC 相关定义（如 RCC_CR_HSEON）
#include "stm32f1xx_hal_can.h"    // CAN 相关定义（如 CAN_TxHeaderTypeDef）
#include "stm32f1xx_hal_uart.h"   // UART 相关定义（如 huart1）
/* Include CMSIS-RTOS header so osSemaphoreId/osThreadId are known */
#include "cmsis_os.h"
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
// 外设句柄外部声明（与 CubeMX 生成的外设文件关联）
/* Peripheral handles provided by CubeMX-generated files */
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* FreeRTOS handles are defined in freertos.c (CubeMX) - declare them here as externs */
extern osSemaphoreId PeriodSemHandle;
extern osSemaphoreId ExtiSemHandle;
extern osThreadId MotorCtrlTaskHandle;
extern osThreadId UartSendTaskHandle;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
