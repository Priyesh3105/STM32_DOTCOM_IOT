/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "string.h"
#include <stdio.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*********** Create Simple Mutex Handler ***************/
SemaphoreHandle_t SimpleMutex;
SemaphoreHandle_t binSemaphore;

/*********** Create Task Handler ***************/
TaskHandle_t HPT_Handler;
TaskHandle_t MPT_Handler;
TaskHandle_t LPT_Handler;

/*********** Create Task functions ***************/
void HPT_Task(void *argument);
void MPT_Task(void *argument);
void LPT_Task(void *argument);

/*********** Create Send_UART functions ***************/
void Send_Uart(char *str) {
	xSemaphoreTake(SimpleMutex, portMAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
	HAL_Delay(1000);
	xSemaphoreGive(SimpleMutex);
}

void Send_Uart2(char *str) {
	xSemaphoreTake(binSemaphore, portMAX_DELAY);
	HAL_Delay(2000);

	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nUsing Semaphore\r\n", 19,
	    				1000);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
	xSemaphoreGive(binSemaphore);
}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*********** Create Simple Mutex ***************/
  	SimpleMutex = xSemaphoreCreateMutex();

  	if (SimpleMutex != NULL)
  		HAL_UART_Transmit(&huart1, (uint8_t*) "Mutex Created\r\n\r\n", 17,
  				1000);
  	else
  		HAL_UART_Transmit(&huart1, (uint8_t*) "Mutex not Created\r\n\r\n", 21,
  				1000);

    /*********** Create Simple binSemaphore ***************/
    	binSemaphore = xSemaphoreCreateBinary();

    	if (SimpleMutex != NULL)
    		HAL_UART_Transmit(&huart1, (uint8_t*) "Semaphore Created\r\n\r\n", 21,
    				1000);
    	else
    		HAL_UART_Transmit(&huart1, (uint8_t*) "Semaphore not Created\r\n\r\n", 25,
    				1000);

    	xSemaphoreGive(binSemaphore);
  	/*********** Create Task ***************/
  	xTaskCreate(HPT_Task, "HPT", 128, NULL, 3, &HPT_Handler);
  	xTaskCreate(MPT_Task, "MPT", 128, NULL, 2, &MPT_Handler);
  	xTaskCreate(LPT_Task, "LPT", 128, NULL, 1, &LPT_Handler);

  	/*********** Create Task Scheduler ***************/
  	vTaskStartScheduler();\

  /* USER CODE END 2 */


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HPT_Task(void *argument) {

	char *strtosend = "\r\n\r\nIN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|\r\n\r\n\r\n";
	while (1) {
		char *str1 = "Entered HPT and about to take Mutex.\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str1, strlen(str1),
				HAL_MAX_DELAY);

		Send_Uart(strtosend);
		Send_Uart2(strtosend);

		char *str2 = "Leaving HPT.\r\n\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str2, strlen(str2),
				HAL_MAX_DELAY);

		vTaskDelay(1000);
	}

}
void MPT_Task(void *argument) {

	char *strtosend = "\r\n\r\nIN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>\r\n\r\n\r\n";
	while (1) {
		char *str1 = "Entered MPT and about to take Mutex.\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str1, strlen(str1),
				HAL_MAX_DELAY);

		Send_Uart(strtosend);
		Send_Uart2(strtosend);


		char *str2 = "Leaving MPT.\r\n\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str2, strlen(str2),
				HAL_MAX_DELAY);

		vTaskDelay(1000);
	}
}
void LPT_Task(void *argument) {

	char *strtosend = "\r\n\r\nIN LPT*-_-*-_-*-_-*-_-*-_-*-_-*\r\n\r\n\r\n";
	while (1) {
		char *str1 = "Entered LPT and about to take Mutex.\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str1, strlen(str1),
				HAL_MAX_DELAY);

		Send_Uart(strtosend);
		Send_Uart2(strtosend);


		char *str2 = "Leaving LPT.\r\n\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str2, strlen(str2),
				HAL_MAX_DELAY);

		vTaskDelay(1000);
	}
}

	/*********** Output of one cycle ***************/


//Mutex Created
//
//Semaphore Created
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Entered MPT and about to take Mutex.
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Leaving MPT.
//
//Entered LPT and about to take Mutex.
//
//
//IN LPT*-_-*-_-*-_-*-_-*-_-*-_-*
//
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Entered MPT and about to take Mutex.
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Leaving MPT.
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Entered MPT and about to take Mutex.
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//
//Using Semaphore
//
//
//IN LPT*-_-*-_-*-_-*-_-*-_-*-_-*
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Entered HPT and about to take Mutex.
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//
//Using Semaphore
//
//
//IN MPT<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>v<^>
//
//
//
//Using Semaphore
//
//
//IN HPT|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|_|-|
//
//
//Leaving HPT.
//
//Leaving MPT.
//
//Leaving LPT.



/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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