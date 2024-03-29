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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
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
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} subKeyBoard;

subKeyBoard keyBoardHIDsub = { 0, 0, 0, 0, 0, 0, 0, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Populate the buffer with the HID report data
		while (1) {
			// Populate the buffer with the HID report data
			keyBoardHIDsub.MODIFIER = 0x02; //print char in capital
			keyBoardHIDsub.KEYCODE1 = 0x13; //print "P"
			keyBoardHIDsub.KEYCODE2 = 0x15; //print "R"
			keyBoardHIDsub.KEYCODE3 = 0x0C; //print "I"
			keyBoardHIDsub.KEYCODE4 = 0x1C; //print "Y"
			keyBoardHIDsub.KEYCODE5 = 0x08; //print "E"
			keyBoardHIDsub.KEYCODE6 = 0x16; //print "S"
			uint8_t buffer[sizeof(keyBoardHIDsub)] = { 0 }; // Initialize the buffer with zeros
			buffer[0] = keyBoardHIDsub.MODIFIER;
			buffer[2] = keyBoardHIDsub.KEYCODE1;
			buffer[3] = keyBoardHIDsub.KEYCODE2;
			buffer[4] = keyBoardHIDsub.KEYCODE3;
			buffer[5] = keyBoardHIDsub.KEYCODE4;
			buffer[6] = keyBoardHIDsub.KEYCODE5;
			buffer[7] = keyBoardHIDsub.KEYCODE6;

			// Send the buffer using USBD_HID_SendReport
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(50);

			// Reset the buffer and send a release report
			memset(buffer, 0, sizeof(buffer));
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(50);

			keyBoardHIDsub.MODIFIER = 0x02; //print char in capital
			keyBoardHIDsub.KEYCODE1 = 0x0B; //print "H"
			keyBoardHIDsub.KEYCODE2 = 0x2C; //print " " a space
			keyBoardHIDsub.KEYCODE3 = 0x16; //print "S"
			keyBoardHIDsub.KEYCODE4 = 0x0B; //print "H"
			keyBoardHIDsub.KEYCODE5 = 0x04; //print "A"
			keyBoardHIDsub.KEYCODE6 = 0x0B; //print "H"
			buffer[0] = keyBoardHIDsub.MODIFIER;
			buffer[2] = keyBoardHIDsub.KEYCODE1;
			buffer[3] = keyBoardHIDsub.KEYCODE2;
			buffer[4] = keyBoardHIDsub.KEYCODE3;
			buffer[5] = keyBoardHIDsub.KEYCODE4;
			buffer[6] = keyBoardHIDsub.KEYCODE5;
			buffer[7] = keyBoardHIDsub.KEYCODE6;

			// Send the buffer using USBD_HID_SendReport
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(50);

			// Reset the buffer and send a release report
			memset(buffer, 0, sizeof(buffer));
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(50);

			keyBoardHIDsub.MODIFIER = 0x02; //print char in capital
			keyBoardHIDsub.KEYCODE1 = 0x0C; //print "I"
			keyBoardHIDsub.KEYCODE2 = 0x28; //print " " enter

			buffer[0] = keyBoardHIDsub.MODIFIER;
			buffer[2] = keyBoardHIDsub.KEYCODE1;
			buffer[3] = keyBoardHIDsub.KEYCODE2;

			// Send the buffer using USBD_HID_SendReport
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(50);

			// Reset the buffer and send a release report
			memset(buffer, 0, sizeof(buffer));
			USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
			HAL_Delay(1000);
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
