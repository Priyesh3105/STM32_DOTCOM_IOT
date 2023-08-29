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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "lsm303agr_reg.h"

/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	void *hbus;
	uint8_t i2c_address;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} sensbus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME              5 //ms
#define TX_BUF_DIM          1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

//uint8_t TX_Buffer [] = "A" ; // DATA to send


static sensbus_t xl_bus = { &SENSOR_BUS, LSM303AGR_I2C_ADD_XL, 0, 0 };

static sensbus_t mag_bus = { &SENSOR_BUS, LSM303AGR_I2C_ADD_MG, 0, 0 };

static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
//static void platform_init(void);
void lsm303agr_read_data_polling(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*-[ I2C Bus Scanning ]-*/



void i2c_scanner(I2C_HandleTypeDef *i2c) {

	printf("Scanning I2C bus:\r\n");
	HAL_StatusTypeDef ret;
	uint16_t i = 0;

	for (i = 1; i < 128; i++) {
		ret = HAL_I2C_IsDeviceReady(i2c, (uint16_t) (i << 1), 3, 10);
		if (ret != HAL_OK) /* No ACK Received At That Address */
		{
			printf("-");
		} else if (ret == HAL_OK) {
			printf("0x%X", i<<1);
		}
	}
	printf("\r\n");
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  i2c_scanner(&hi2c1);
  lsm303agr_read_data_polling();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void lsm303agr_read_data_polling(void) {
	/* Initialize mems driver interface */
	stmdev_ctx_t dev_ctx_xl;
	dev_ctx_xl.write_reg = platform_write;
	dev_ctx_xl.read_reg = platform_read;
	dev_ctx_xl.handle = (void*) &xl_bus;
	stmdev_ctx_t dev_ctx_mg;
	dev_ctx_mg.write_reg = platform_write;
	dev_ctx_mg.read_reg = platform_read;
	dev_ctx_mg.handle = (void*) &mag_bus;
	/* Wait boot time and initialize platform specific hardware */
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);
	/* Check device ID */
	whoamI = 0;
	lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);  // who_iamid store 51 so check device id 3 means 51

	if (whoamI != LSM303AGR_ID_XL)
		while (1)
			; /*manage here device not found */

	whoamI = 0;
	lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);

	if (whoamI != LSM303AGR_ID_MG)
		while (1)
			; /*manage here device not found */

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

	do {
		lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
	lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
	lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
	/* Set accelerometer full scale */
	lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
			LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Enable temperature sensor */
	lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
	/* Set device in continuous mode */
	lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
	/* Set magnetometer in continuous mode */
	lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);

	/* Read samples in polling mode (no int) */
	while (1) {
		/* Read output only if new value is available */
		lsm303agr_reg_t reg;
		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

		if (reg.status_reg_a.zyxda) {
			/* Read accelerometer data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration);
			acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration[0]);
			acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration[1]);
			acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
					data_raw_acceleration[2]);
			sprintf((char*) tx_buffer,
					"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
					acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//			printf(tx_buffer, strlen((char const*) tx_buffer));
			printf("%s\r\n",tx_buffer);
		}

		lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);

		if (reg.status_reg_m.zyxda) {
			/* Read magnetic field data */
			memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
			lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic);
			magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[0]);
			magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[1]);
			magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[2]);
			sprintf((char*) tx_buffer,
					"Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
					magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
			//printf(tx_buffer, strlen((char const*) tx_buffer));
		}

		lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);

		if (reg.byte) {
			/* Read temperature data */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lsm303agr_temperature_raw_get(&dev_ctx_xl, &data_raw_temperature);
			temperature_degC = lsm303agr_from_lsb_hr_to_celsius(
					data_raw_temperature);
			sprintf((char*) tx_buffer, "Temperature [degC]:%6.2f\r\n",
					temperature_degC);
			tx_com(tx_buffer, strlen((char const*) tx_buffer));
		}
	}
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		reg |= 0x80;
	}

	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);


//	HAL_I2C_Mem_Write_IT(sensbus->hbus, sensbus->i2c_address, reg,
//			I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len);





	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		reg |= 0x80;
	}

	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

//	HAL_I2C_Mem_Read_IT(sensbus->hbus, sensbus->i2c_address, reg,
//			I2C_MEMADD_SIZE_8BIT, bufp, len);






//	HAL_I2C_Master_Receive_IT(sensbus->hbus, sensbus->i2c_address, bufp, len);

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len) {
//#if defined(NUCLEO_F411RE)
 // HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
//#elif defined(STEVAL_MKI109V3)
//  CDC_Transmit_FS(tx_buffer, len);
//#elif defined(SPC584B_DIS)
//  sd_lld_write(&SD2, tx_buffer, len);
//#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */


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
