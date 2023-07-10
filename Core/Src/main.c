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
//#include "bmp180_for_stm32_hal.h"
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
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define I2C_SCAN_MAX_DELAY 15
void I2C_Scan() {
	HAL_StatusTypeDef res;
	char info[] = "Scanning I2C bus...\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) info, strlen(info),
			I2C_SCAN_MAX_DELAY);
	for (uint16_t i = 0; i < 128; i++) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, I2C_SCAN_MAX_DELAY);
		if (res == HAL_OK) {
			char msg[64];
			snprintf(msg, sizeof(msg), "0x%02X", i);
			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
			I2C_SCAN_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2,
					I2C_SCAN_MAX_DELAY);
		}
		//else HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, I2C_SCAN_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nend\r\n", 7, I2C_SCAN_MAX_DELAY);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t map(float x, int16_t in_min, int16_t in_max, int16_t out_min,
		int16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#define constrain(x,u,d) (u>d?(x>u?u:(x<d?d:x)):(x>d?d:(x<u?u:x)))
#define max(a,b) (a>b)?a:b
void blink_stmled();
void blink_rfled(uint8_t count);
static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout);
readings data;
static HAL_StatusTypeDef I2CResetBus(void) {
	__HAL_I2C_DISABLE(&hi2c1);
	/* 1. Set SWRST bit in I2Cx_CR1 register. */
	hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
	HAL_Delay(2);
	/* 2. Clear SWRST bit in I2Cx_CR1 register. */
	hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
	HAL_Delay(2);
	/* 3. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
	MX_I2C1_Init();
	__HAL_I2C_ENABLE(&hi2c1);
	HAL_Delay(2);
	hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
	hi2c1.State = HAL_I2C_STATE_READY;
	hi2c1.PreviousState = 0;
	hi2c1.Mode = HAL_I2C_MODE_NONE;
	return HAL_OK;
}
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
	//I2CResetBus();
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(150);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	blink_stmled();
	debug_init(&huart1);
	log_s("Start");
	//I2C_Scan();
	gy_init_lbl: if (GY801_init(&hi2c1, &data)) {
		I2C_ClearBusyFlagErratum(&hi2c1, 100); //fix i2c HAL_BUSY: https://istarik.ru/blog/stm32/123.html
		log_s("REINIT_DONE TRYING....");
		goto gy_init_lbl;
		blink_rfled(1); //HAL_GPIO_WritePin(GPIOC, LED_RF_Pin, GPIO_PIN_SET);
	}

	uint8_t nrf_data[32];
	nrf_init(&hspi1);
	uint32_t packet = 0;

	extern char SDPath[4];
	FATFS fileSystem;
	uint8_t testbuff[] = "new run\r\n";
	char sdBuff[256];
	FIL logFile;
	UINT tempBytes;
	UINT sdSize = 0;
	FRESULT res = 0;
	FSIZE_t fsize;
	if (f_mount(&fileSystem, (TCHAR const*) SDPath, 0) == FR_OK) {
		if (f_open(&logFile, "log.txt", FA_WRITE | FA_OPEN_ALWAYS) == FR_OK) {
			uint64_t fsize = f_size(&logFile);
			f_lseek(&logFile, fsize);
			if (f_write(&logFile, testbuff, sizeof(testbuff),
					(void*) &tempBytes) == FR_OK) {
				f_sync(&logFile);
			} else {
				log_s("SD_ERR_WRITE");
				blink_rfled(2);
				goto sd_end_lbl;
			}
		} else {
			log_s("SD_ERR_OPEN");
			blink_rfled(2);
			goto sd_end_lbl;
		}
	} else {
		log_s("SD_ERR_MOUNT");
		blink_rfled(2);
		goto sd_end_lbl;
	}
	sd_end_lbl: ;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint32_t last_t; //,test_t;
	float pitch, roll;
	float max_height=0,start_height=0;
	uint32_t max_height_t = 0,max_accel_t=0;
	uint8_t start_flag=0,end_flag=0,motor_flag=0,enable_compensate_flag=0;

	log_s_int("con 0", constrain(2, -1, 1));
	log_s_int("con 2", constrain(2, 1, -1));
	log_s_int("con -10", constrain(-10, -1, 1));
	// madgwick
	log_s_p_3("ADXL", &data.adxl345.ax, &data.adxl345.ay, &data.adxl345.az);
	log_s_p_3("ACCEL", &data.lsm303dlhc.ax, &data.lsm303dlhc.ay,
			&data.lsm303dlhc.az);
	log_s_p_3("GYRO", &data.l3g4200d.gx, &data.l3g4200d.gy, &data.l3g4200d.gz);
	log_s_p_3("MAG", &data.lsm303dlhc_mag.mx_raw, &data.lsm303dlhc_mag.my_raw,
			&data.lsm303dlhc_mag.mz_raw);
	uint32_t ahrs_t = HAL_GetTick();
	uint16_t hz = 0;
	uint32_t hz_t = HAL_GetTick();
	log_s_p("HZ", &hz);
	mahony_print_ptr();
	//char *log_chars[50];
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (!adxl_check_connection()) {
			I2C_ClearBusyFlagErratum(&hi2c1, 100);
		}
		GY801_update_data();
		hz += 1;
		mahony_update((float) (HAL_GetTick() - ahrs_t) / 1000.0f,
				(float) data.l3g4200d.gx * M_PI / 180 / 131,
				(float) data.l3g4200d.gy * M_PI / 180 / 131,
				(float) data.l3g4200d.gz * M_PI / 180 / 131,
				(float) data.adxl345.ay * 9.81 / 32.0f,
				(float) data.adxl345.ax * 9.81 / 32.0f,
				(float) data.adxl345.az * 9.81 / 32.0f,
				(float) data.lsm303dlhc_mag.mx / 450,
				(float) data.lsm303dlhc_mag.my / 450,
				(float) data.lsm303dlhc_mag.mz / 450);
		//log_s_int("AHRS_T", HAL_GetTick() - ahrs_t);
		ahrs_t = HAL_GetTick();
		pitch = (enable_compensate_flag==1)?constrain(mahony_getPitch(),-20,20):0  + 10;
		roll = (enable_compensate_flag==1)?constrain(mahony_getRoll(),-20,20):0 + 10;
		htim2.Instance->CCR1 = constrain(
				map(/*(pitch<15+21 && pitch> -15+21)?pitch:21*/pitch, -90, 90,
						125, 25), 25, 125);
		htim3.Instance->CCR1 = constrain(
				map(/*(roll<15-23 && roll> -15-23)?roll:-23*/roll, -90, 90, 25,
						125), 25, 125);

		if (HAL_GetTick() - hz_t >= 1000) {
			hz_t = HAL_GetTick();
			log_s_int("HZ", hz);
			hz = 0;
		}

		if (HAL_GetTick() - last_t >= 100) {
			last_t = HAL_GetTick();
			BMP180_upd_data();
			nrf_data[0] = (packet >> 8 * 0) & 0xFF;
			nrf_data[1] = (packet >> 8 * 1) & 0xFF;
			nrf_data[2] = (packet >> 8 * 2) & 0xFF;
			nrf_data[3] = (packet >> 8 * 3) & 0xFF;
			nrf_data[4] = ((uint16_t) (data.bmp180.pressure / 10) >> 8 * 0)
					& 0xFF;
			nrf_data[5] = ((uint16_t) (data.bmp180.pressure / 10) >> 8 * 1)
					& 0xFF;
			nrf_data[6] = (data.adxl345.ay >> 8 * 0) & 0xFF;
			nrf_data[7] = (data.adxl345.ay >> 8 * 1) & 0xFF;
			nrf_data[8] = (data.adxl345.ax >> 8 * 0) & 0xFF;
			nrf_data[9] = (data.adxl345.ax >> 8 * 1) & 0xFF;
			nrf_data[10] = (data.adxl345.az >> 8 * 0) & 0xFF;
			nrf_data[11] = (data.adxl345.az >> 8 * 1) & 0xFF;
			nrf_data[12] = (data.l3g4200d.gx >> 8 * 0) & 0xFF;
			nrf_data[13] = (data.l3g4200d.gx >> 8 * 1) & 0xFF;
			nrf_data[14] = (data.l3g4200d.gy >> 8 * 0) & 0xFF;
			nrf_data[15] = (data.l3g4200d.gy >> 8 * 1) & 0xFF;
			nrf_data[16] = (data.l3g4200d.gz >> 8 * 0) & 0xFF;
			nrf_data[17] = (data.l3g4200d.gz >> 8 * 1) & 0xFF;
			nrf_data[18] = (data.lsm303dlhc_mag.mx_raw >> 8 * 0) & 0xFF;
			nrf_data[19] = (data.lsm303dlhc_mag.mx_raw >> 8 * 1) & 0xFF;
			nrf_data[20] = (data.lsm303dlhc_mag.my_raw >> 8 * 0) & 0xFF;
			nrf_data[21] = (data.lsm303dlhc_mag.my_raw >> 8 * 1) & 0xFF;
			nrf_data[22] = (data.lsm303dlhc_mag.mz_raw >> 8 * 0) & 0xFF;
			nrf_data[23] = (data.lsm303dlhc_mag.mz_raw >> 8 * 1) & 0xFF;
			nrf_data[24] = ((int16_t) (mahony_getRoll() * 100) >> 8 * 0) & 0xFF;
			nrf_data[25] = ((int16_t) (mahony_getRoll() * 100) >> 8 * 1) & 0xFF;
			nrf_data[26] = ((int16_t) (mahony_getPitch() * 100) >> 8 * 0)
					& 0xFF;
			nrf_data[27] = ((int16_t) (mahony_getPitch() * 100) >> 8 * 1)
					& 0xFF;
			nrf_data[28] = ((int16_t) (mahony_getYaw() * 100) >> 8 * 0) & 0xFF;
			nrf_data[29] = ((int16_t) (mahony_getYaw() * 100) >> 8 * 1) & 0xFF;
			nrf_data[30] = (data.bmp180.temp >> 8 * 0) & 0xFF;
			nrf_data[31] = (data.bmp180.temp >> 8 * 1) & 0xFF;
			//uint32_t test_t=HAL_GetTick();
			nrf_send(nrf_data);  //2ms
			//log_s_int("NRF_T",HAL_GetTick()-test_t);

			memset(sdBuff, 0, sizeof(sdBuff));
			sprintf(sdBuff,
					"%ld|%ld|%f|%.1f|%ld|%f|%d|%d|%d|%d|%d|%d|%f|%f|%f|%f|%f|%f|%d|%d|%d|\r\n",
					packet, HAL_GetTick(), (float) data.bmp180.height,
					((float) data.bmp180.temp) / 10, data.bmp180.pressure,
					sqrt(pow(data.adxl345.ax,2)+ pow(data.adxl345.ay,2)+ pow(data.adxl345.az,2)),
					data.adxl345.ax, data.adxl345.ay, data.adxl345.az,
					data.l3g4200d.gx, data.l3g4200d.gy, data.l3g4200d.gz,
					(float) data.lsm303dlhc_mag.mx,
					(float) data.lsm303dlhc_mag.my,
					(float) data.lsm303dlhc_mag.mz,
					mahony_getRoll(), mahony_getPitch(), mahony_getYaw(),
					start_flag,motor_flag,end_flag);
			//log_s_wnl(sdBuff);
			if (f_write(&logFile, sdBuff, strlen(sdBuff), &tempBytes)
					== FR_OK) {
				f_sync(&logFile);
			} else
				HAL_GPIO_TogglePin(GPIOC, LED_RF_Pin);
			/*sprintf((char*) log_chars, "|%c%c|\r\n",(data.adxl345.ax >> 8 * 0) & 0xFF,(data.adxl345.ax >> 8 * 1) & 0xFF);
			 log_s_wnl((const char*) log_chars);*/
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (packet>50 && packet<100)?GPIO_PIN_SET:GPIO_PIN_RESET);
			if(sqrt(pow(data.adxl345.ax,2)+ pow(data.adxl345.ay,2)+ pow(data.adxl345.az,2))>32*2 && max_accel_t==0 && HAL_GetTick() > 2000){
				max_accel_t=HAL_GetTick();
			}
			if(HAL_GetTick()-max_accel_t>2000 && max_accel_t!=0){
				enable_compensate_flag=1;
			}
			if (HAL_GetTick() > 5000) {
				if(start_height==0){
					start_height=data.bmp180.height;
				}
				if (data.bmp180.height-start_height>10){
					start_flag=1;
				}
				if(start_flag && data.bmp180.height-start_height<10){
					end_flag=1;
				}
				//log_s_float("height", data.bmp180.height);
				max_height = max(data.bmp180.height, max_height);
				//log_s_float("m__eight", max_height);
				if (max_height - data.bmp180.height > 5) {
					if (max_height_t == 0) {
						max_height_t = HAL_GetTick();
						motor_flag=1;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
					}
					if (HAL_GetTick()-max_height_t  > 5000
							&& max_height_t != 0) {
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
					}
				}
			}
			packet++;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

		}

		//HAL_Delay(20);
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 14;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1680 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1680 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED_MCU_Pin | GPIO_PIN_1 | LED_RF_Pin | GPIO_PIN_4,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_MCU_Pin PC1 LED_RF_Pin PC4 */
	GPIO_InitStruct.Pin = LED_MCU_Pin | GPIO_PIN_1 | LED_RF_Pin | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SDIO_CD_Pin IRQ_Pin */
	GPIO_InitStruct.Pin = SDIO_CD_Pin | IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* I2C fix */
	/*GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	 for (int i = 0; i<10; i++) {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	 HAL_Delay(3);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	 HAL_Delay(3);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	 HAL_Delay(3);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	 HAL_Delay(3);
	 }
	 GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);*/
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void blink_stmled() {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}

void blink_rfled(uint8_t count) {
	for (uint8_t i = 0; i < count; i++) {
		HAL_GPIO_TogglePin(GPIOC, LED_RF_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, LED_RF_Pin);
		HAL_Delay(100);
	}
}

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin,
		GPIO_PinState state, uint32_t timeout) {
	uint32_t Tickstart = HAL_GetTick();
	uint8_t ret = 1;

	for (; (state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
			{
		if (timeout != HAL_MAX_DELAY) // Check for the timeout
		{
			if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
				ret = 0;
		}

		asm("nop");
	}
	return ret;
}
static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout) {
	/**I2C1 GPIO Configuration
	 PB6     ------> I2C1_SCL
	 PB7     ------> I2C1_SDA
	 */
	// 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };

	// 1. Clear PE bit.
	CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_I2C_DeInit(hi2c);

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	GPIO_InitStructure.Pin = GPIO_PIN_6; // SCL // если пин другой, то укажите нужный
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); // если порт другой, то укажите нужную букву GPIOх, и ниже там все порты и пины поменяйте на своё

	GPIO_InitStructure.Pin = GPIO_PIN_7; // SDA
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 3. Check SCL and SDA High level in GPIOx_IDR.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_SET, timeout);
	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_SET, timeout);

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	// 5. Check SDA Low level in GPIOx_IDR.
	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET, timeout);

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	// 7. Check SCL Low level in GPIOx_IDR.
	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET, timeout);

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_SET, timeout);

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_SET, timeout);

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	//GPIO_InitStructure.Alternate = GPIO_AF4_I2C2; // F4

	GPIO_InitStructure.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
	asm("nop");

	/* 14. Clear SWRST bit in I2Cx_CR1 register. */
	CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
	asm("nop");

	/* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
	SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
	asm("nop");

	// Call initialization function.
	HAL_I2C_Init(hi2c);
}
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
