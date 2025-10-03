/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"  // 包含 HAL 库总头文件，里面会包含大部分外设相关头文件
#include "stm32f1xx_hal_rcc.h"  // 明确包含 RCC 相关头文件
#include "stm32f1xx_hal_can.h"  // 明确包含 CAN 相关头文件

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t g_motor_target_angle = 0;  // 电机目标角度（0° 或 360°）
uint32_t g_current_period = 2000;   // 当前周期（ms），默认 2000ms
uint32_t g_motor_actual_angle = 0;  // 电机实际角度（从 CAN 接收）

/* FreeRTOS 信号量句柄（需和 CubeMX 配置一致） */
osSemaphoreId PeriodSemHandle;
osSemaphoreId ExtiSemHandle;

/* FreeRTOS 任务句柄 */
osThreadId MotorCtrlTaskHandle;
osThreadId UartSendTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void M2006_Send_Pos_Cmd(uint16_t angle, uint8_t id);  // M2006 位置控制指令发送
void MotorCtrlTask(void const *argument);             // 电机控制任务
void UartSendTask(void const *argument);              // 串口发送任务
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 启动定时器（周期中断）
  HAL_TIM_Base_Start_IT(&htim2);
  // 发送初始角度指令（0°）
  M2006_Send_Pos_Cmd(0, 0x201);  // 假设电机 ID 为 0x201
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// CAN 滤波配置 + M2006 指令发送 + CAN 接收回调
void M2006_Send_Pos_Cmd(uint16_t angle, uint8_t id)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;

    // 角度转编码值（M2006 分辨率：8192 码/圈 → 360°=8192 码）
    int16_t encode = angle * 8192 / 360;

    tx_header.StdId = id;          // 电机 ID（M2006 默认 0x201~0x204，按实际修改）
    tx_header.RTR = CAN_RTR_DATA;  // 数据帧
    tx_header.IDE = CAN_ID_STD;    // 标准 ID
    tx_header.DLC = 8;             // 数据长度 8 字节
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = encode & 0xFF;                // 编码低 8 位
    tx_data[1] = (encode >> 8) & 0xFF;         // 编码高 8 位
    tx_data[2] = 0;                            // 保留（速度限制，默认 0）
    tx_data[3] = 0;                            // 保留（电流限制，默认 0）
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0); // 等待发送完成
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};

    if (hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        // 只处理 M2006 反馈（ID=0x201）
        if (rx_header.StdId == 0x201)
        {
            // 接收编码值转角度（8192 码=360°）
            int16_t encode = (rx_data[1] << 8) | rx_data[0];
            g_motor_actual_angle = encode * 360 / 8192;
        }
    }
}

// 定时器周期中断回调（释放信号量，通知电机任务切换位置）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        osSemaphoreRelease(PeriodSemHandle); // 释放周期信号量
    }
}

// 外部中断回调（切换运动周期）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) // 假设外部中断引脚为 PA0
    {
        osSemaphoreRelease(ExtiSemHandle); // 释放外部中断信号量
        // 切换周期（2s ↔ 1s），更新定时器自动重装载值
        if (g_current_period == 2000)
        {
            g_current_period = 1000;
            __HAL_TIM_SET_AUTORELOAD(&htim2, 9999); // 10kHz/(9999+1)=1Hz → 1s
        }
        else
        {
            g_current_period = 2000;
            __HAL_TIM_SET_AUTORELOAD(&htim2, 19999); // 2s
        }
        __HAL_TIM_SET_COUNTER(&htim2, 0); // 重置计数器
    }
}

// 电机控制任务（FreeRTOS 任务函数）
void MotorCtrlTask(void const *argument)
{
    for (;;)
    {
        // 等待定时器信号量（2s/1s 触发一次）
        osSemaphoreWait(PeriodSemHandle, osWaitForever);
        
        // 切换目标角度（0° ↔ 360°）
        if (g_motor_target_angle == 0)
        {
            g_motor_target_angle = 360;
        }
        else
        {
            g_motor_target_angle = 0;
        }
        
        // 发送目标角度指令给 M2006
        M2006_Send_Pos_Cmd(g_motor_target_angle, 0x201);
        
        osDelay(10); // 轻微延时，避免任务占用过高 CPU
    }
}

// 串口发送任务（FreeRTOS 任务函数）
void UartSendTask(void const *argument)
{
    char uart_buf[50]; // 串口发送缓存
    for (;;)
    {
        // 格式化电机实际角度（50Hz：20ms 发送一次）
        sprintf(uart_buf, "Motor Actual Angle: %d°\r\n", g_motor_actual_angle);
        // 发送数据（阻塞发送，确保数据完整）
        HAL_UART_Transmit(&husart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        
        osDelay(20); // 20ms → 50Hz 频率
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
