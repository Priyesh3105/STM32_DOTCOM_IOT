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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
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
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

char buffer[1024]; //store the data

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


// to send the uart

void send_uart(char *string){
	uint8_t len=strlen(string);
	HAL_UART_Transmit(&hlpuart1,(uint8_t *)string,len,2000); // transmiting in bloking mode

}

// to fined the size of data in buffer

int bufsize(char *buf){
	int i=0;
	while(*buf++ != '\0');
	return i;
}

void bufclear(void){  // clear the buffer

	for(int i=0;i<1024; i++){
		buffer[i]='\0';
	}
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
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  //Mount sd card

   fresult = f_mount(&fs, "/", 1);
   if (fresult != FR_OK)
 	   send_uart ("ERROR!!! in mounting SD CARD...\r\n");
   else send_uart("SD CARD mounted successfully...\r\n");


   /*************** Card capacity details ********************/

     	/* Check free space */
     	f_getfree("", &fre_clust, &pfs);

     	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
     	sprintf (buffer, "SD CARD Total Size: \t%lu\r\n",total);
     	send_uart(buffer);
     	bufclear();
     	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
     	sprintf (buffer, "SD CARD Free Space: \t%lu\r\n",free_space);
     	send_uart(buffer);
     	bufclear();


//     	//puts and gets operation
//
//
//     	/* Create file with read write access and open it */
//     	fresult=f_open(&fil,"file1.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
//
//     	//Writing text
//
//     	fresult=f_puts("This is First File\r\n",&fil);
//
//
//     	//Close file
//
//     	fresult=f_close(&fil);
//     	send_uart("File1.txt is opened and it contains the data as shown below\n");
//
//
//
//     	/* Open file to read */
//           	fresult = f_open(&fil, "file1.txt", FA_READ);
//
//
//           	/* Read string from the file */
//           	f_gets(buffer, f_size(&fil), &fil);
//
//           	send_uart(buffer) ;
//           	//Close file
//
//           	   f_close(&fil);

 //

 //
 //      //operation using f_write and f_read
 //
 //
 //     /* Create file with read write access and open it */
 //
 //     fresult=f_open(&fil,"file2.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
 //
 //     // Writing text
 //
 //     strcpy(buffer,"This file 2 and it says hello Rahul ");
 //
 //     fresult=f_write(&fil, buffer,bufsize(buffer),&bw);
 //
 //    send_uart("file2 created and written data\n");
 //
 ////    close file
 //
 //    f_close(&fil);
 //
 //// clear the buffer to show that result obtaeiend is from the file
 //
 //    bufclear();
 //
 ////    open secon file to read
 //
 //    fresult=f_open(&fil,"file2.txt",FA_READ);
 //
 //
 ////    Read the data from the file
 //
 //    f_read(&fil, buffer,f_size(&fil),&br);
 //    send_uart(buffer);
 //
 //
 ////    Close file
 //    f_close(&fil);
 //
 //    bufclear();




           	  	/**************** The following operation is using f_write and f_read **************************/

           	  	/* Create second file with read write access and open it */
           	  	fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);

           	  	/* Writing text */
           	  	strcpy (buffer, "This is File2.txt, written using ...f_write... and it says Hello from Priyesh \r\n");

           	  	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

           	  	send_uart ("File2.txt created and data is written\r\n");

           	  	/* Close file */
           	  	f_close(&fil);



           	  	// clearing buffer to show that result obtained is from the file
 //          	    bufclear();

           	  	/* Open second file to read */
           	  	fresult = f_open(&fil, "file2.txt", FA_READ);



           	  	if (fresult == FR_OK){
           	  		send_uart ("file2.txt is open and the data is shown below\r\n");

           	  	}
           	  /* Read data from the file
           	            	     * Please see the function details for the arguments */
           	   f_read (&fil, buffer, f_size(&fil), &br);

           		send_uart(buffer);
           	    send_uart("\r\n");



           	  	/* Close file */
           	  	f_close(&fil);

           	     bufclear();


 //          	  	/*********************UPDATING an existing file ***************************/
 //
 //          	  	/* Open the file with write access */
 //          	  	fresult = f_open(&fil, "file2.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
 //
 //          	  	/* Move to offset to the end of the file */
 //          	  	fresult = f_lseek(&fil, f_size(&fil));
 //
 //          	  	if (fresult == FR_OK)send_uart ("About to update the file2.txt\n");
 //
 //          	  	/* write the string to the file */
 //          	  	fresult = f_puts("This is updated data and it should be in the end", &fil);
 //
 //          	  	f_close (&fil);
 //
 //          	    bufclear();
 //
 //          	  	/* Open to read the file */
 //          	  	fresult = f_open (&fil, "file2.txt", FA_READ);
 //
 //          	  	/* Read string from the file */
 //          	  	fresult = f_read (&fil, buffer, f_size(&fil), &br);
 //          	  	if (fresult == FR_OK)send_uart ("Below is the data from updated file2.txt\n");
 //          	  	send_uart(buffer);
 //          	  	send_uart("\n\n");
 //
 //          	  	/* Close file */
 //          	  	f_close(&fil);
 //
 //          	    bufclear();
 //
 //
 //          	  	/*************************REMOVING FILES FROM THE DIRECTORY ****************************/
 //
 //          	  	fresult = f_unlink("/file1.txt");
 //          	  	if (fresult == FR_OK) send_uart("file1.txt removed successfully...\n");
 //
 //          	  	fresult = f_unlink("/file2.txt");
 //          	  	if (fresult == FR_OK) send_uart("file2.txt removed successfully...\n");
 //
 //          	  	/* Unmount SDCARD */
 //          	  	fresult = f_mount(NULL, "/", 1);
 //          	  	if (fresult == FR_OK) send_uart ("SD CARD UNMOUNTED successfully...\n");

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
