/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Provide weak default implementations for IRQ handlers referenced by the
  startup vector. These are weak so user code can override them; they
  simply loop if not overridden. */
void WWDG_IRQHandler(void) __attribute__ ((weak));
void WWDG_IRQHandler(void) { while (1) {} }
void PVD_IRQHandler(void) __attribute__ ((weak));
void PVD_IRQHandler(void) { while (1) {} }
void TAMPER_IRQHandler(void) __attribute__ ((weak));
void TAMPER_IRQHandler(void) { while (1) {} }
void RTC_IRQHandler(void) __attribute__ ((weak));
void RTC_IRQHandler(void) { while (1) {} }
void FLASH_IRQHandler(void) __attribute__ ((weak));
void FLASH_IRQHandler(void) { while (1) {} }
void RCC_IRQHandler(void) __attribute__ ((weak));
void RCC_IRQHandler(void) { while (1) {} }
void EXTI0_IRQHandler(void) __attribute__ ((weak));
void EXTI0_IRQHandler(void) { while (1) {} }
void EXTI1_IRQHandler(void) __attribute__ ((weak));
void EXTI1_IRQHandler(void) { while (1) {} }
void EXTI2_IRQHandler(void) __attribute__ ((weak));
void EXTI2_IRQHandler(void) { while (1) {} }
void EXTI3_IRQHandler(void) __attribute__ ((weak));
void EXTI3_IRQHandler(void) { while (1) {} }
void EXTI4_IRQHandler(void) __attribute__ ((weak));
void EXTI4_IRQHandler(void) { while (1) {} }
void DMA1_Channel1_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel1_IRQHandler(void) { while (1) {} }
void DMA1_Channel2_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel2_IRQHandler(void) { while (1) {} }
void DMA1_Channel3_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel3_IRQHandler(void) { while (1) {} }
void DMA1_Channel4_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel4_IRQHandler(void) { while (1) {} }
void DMA1_Channel5_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel5_IRQHandler(void) { while (1) {} }
void DMA1_Channel6_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel6_IRQHandler(void) { while (1) {} }
void DMA1_Channel7_IRQHandler(void) __attribute__ ((weak));
void DMA1_Channel7_IRQHandler(void) { while (1) {} }
void ADC1_2_IRQHandler(void) __attribute__ ((weak));
void ADC1_2_IRQHandler(void) { while (1) {} }
void USB_HP_CAN1_TX_IRQHandler(void) __attribute__ ((weak));
void USB_HP_CAN1_TX_IRQHandler(void) { while (1) {} }
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__ ((weak));
void USB_LP_CAN1_RX0_IRQHandler(void) { while (1) {} }
void CAN1_RX1_IRQHandler(void) __attribute__ ((weak));
void CAN1_RX1_IRQHandler(void) { while (1) {} }
void CAN1_SCE_IRQHandler(void) __attribute__ ((weak));
void CAN1_SCE_IRQHandler(void) { while (1) {} }
void EXTI9_5_IRQHandler(void) __attribute__ ((weak));
void EXTI9_5_IRQHandler(void) { while (1) {} }
void TIM1_BRK_IRQHandler(void) __attribute__ ((weak));
void TIM1_BRK_IRQHandler(void) { while (1) {} }
void TIM1_UP_IRQHandler(void) __attribute__ ((weak));
void TIM1_UP_IRQHandler(void) { while (1) {} }
void TIM1_TRG_COM_IRQHandler(void) __attribute__ ((weak));
void TIM1_TRG_COM_IRQHandler(void) { while (1) {} }
void TIM1_CC_IRQHandler(void) __attribute__ ((weak));
void TIM1_CC_IRQHandler(void) { while (1) {} }
void TIM2_IRQHandler(void) __attribute__ ((weak));
void TIM2_IRQHandler(void) { while (1) {} }
void TIM3_IRQHandler(void) __attribute__ ((weak));
void TIM3_IRQHandler(void) { while (1) {} }
void TIM4_IRQHandler(void) __attribute__ ((weak));
void TIM4_IRQHandler(void) { while (1) {} }
void I2C1_EV_IRQHandler(void) __attribute__ ((weak));
void I2C1_EV_IRQHandler(void) { while (1) {} }
void I2C1_ER_IRQHandler(void) __attribute__ ((weak));
void I2C1_ER_IRQHandler(void) { while (1) {} }
void I2C2_EV_IRQHandler(void) __attribute__ ((weak));
void I2C2_EV_IRQHandler(void) { while (1) {} }
void I2C2_ER_IRQHandler(void) __attribute__ ((weak));
void I2C2_ER_IRQHandler(void) { while (1) {} }
void SPI1_IRQHandler(void) __attribute__ ((weak));
void SPI1_IRQHandler(void) { while (1) {} }
void SPI2_IRQHandler(void) __attribute__ ((weak));
void SPI2_IRQHandler(void) { while (1) {} }
void USART1_IRQHandler(void) __attribute__ ((weak));
void USART1_IRQHandler(void) { while (1) {} }
void USART2_IRQHandler(void) __attribute__ ((weak));
void USART2_IRQHandler(void) { while (1) {} }
void USART3_IRQHandler(void) __attribute__ ((weak));
void USART3_IRQHandler(void) { while (1) {} }
void EXTI15_10_IRQHandler(void) __attribute__ ((weak));
void EXTI15_10_IRQHandler(void) { while (1) {} }
void RTC_Alarm_IRQHandler(void) __attribute__ ((weak));
void RTC_Alarm_IRQHandler(void) { while (1) {} }
void USBWakeUp_IRQHandler(void) __attribute__ ((weak));
void USBWakeUp_IRQHandler(void) { while (1) {} }

