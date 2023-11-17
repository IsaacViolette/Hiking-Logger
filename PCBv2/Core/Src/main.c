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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32l4xx.h"
#include "GPS.h"
#include "I2C.h"
#include "ssd1306.h"
#include "lis3dh.h"
#include "count_steps.h"
#include "math.h"   //using this for converting the CSV data from float to int
#include <stdarg.h>

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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Allocate a buffer for reading data from the sensor.
// Six bytes required to read XYZ data.
uint8_t xyz_buf[6] = { 0 };
// New instance of the lis3dh convenience object.
lis3dh_t lis3dh;

// lis3dh calls return this HAL status type.
HAL_StatusTypeDef status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t nmea;
char nmea_buf[256];
char nmea_gga[256];
uint8_t i = 0;

double cur_lat = 0;
double cur_lon = 0;
double pre_lat = 0;
double pre_lon = 0;
char cur_time[9];
char pre_time[9];
double alt = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {

        nmea_buf[i++] = nmea;

        if (nmea == '\n' || i >= sizeof(nmea_buf) - 1) {
        	if(nmea_buf[3]=='G' && nmea_buf[4]=='G' && nmea_buf[5] == 'A')
        	{
        		memcpy(nmea_gga, nmea_buf, 256);
        		cur_lat = get_lat(nmea_gga);
        		cur_lon = get_lon(nmea_gga);
        		get_time(nmea_gga, pre_time);
        		alt = get_alt(nmea_gga);
        	}

            memset(nmea_buf, 0, sizeof(nmea_buf));
            i = 0;
        }

        // Start the next reception
        HAL_UART_Receive_IT(&huart1, &nmea, 1);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	UINT bytesWrote;
	FRESULT fres;
	BYTE writeBuf1[33];

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();

	/* Create new file on MicroSD */
	HAL_Delay(1000);
	f_mount(&FatFs, "", 1); //1=mount now
	f_open(&fil, "crds.txt", FA_CREATE_ALWAYS | FA_OPEN_ALWAYS);
	f_close(&fil);
	f_mount(NULL, "", 0); //0=demount

	char buf1[16];
	char sd1[33];
	char message[64] = "Starting Up";

	status = lis3dh_init(&lis3dh, &hi2c1, xyz_buf, 6);
	if (status != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(2,0);
	ssd1306_WriteString(message, Font_11x18, White);
	ssd1306_UpdateScreen();

	uint16_t  num_steps  = 0;
	float total_distance = 0;
	float new_distance;
	float miles;
	HAL_UART_Receive_IT(&huart1, &nmea, 1);

	while (1)
	{
		//hold the data from the CSV file in a fifo-like data structure where the accelerometer data looks like
		//[x1,y1,z1,x2,y2,z2...x400,y400,z400]
		int8_t acc[NUM_SAMPLES*3] = {0};
		uint16_t i    = 0;
		float    temp = 0;
		while(i < NUM_SAMPLES*3) //while data array is being filled
		{
			HAL_Delay(50); //20Hz

			//scaling factor to convert the decimal data to int8 integers. calculated in matlab by taking the absolute value of all the data
			//and then calculating the max of that data. then divide that by 127 to get the scaling factor
			float scale_factor = 55.3293;

			if(lis3dh_xyz_available(&lis3dh)) {
				status = lis3dh_get_xyz(&lis3dh);
				float xx = lis3dh.x/ACCEL_DATA_SCALER;
				float yy = lis3dh.y/ACCEL_DATA_SCALER;
				float zz = lis3dh.z/ACCEL_DATA_SCALER;

				temp     = roundf(xx*scale_factor);
				acc[i++] = (int8_t)temp;

				temp     = roundf(yy*scale_factor);
				acc[i++] = (int8_t)temp;

				temp     = roundf(zz*scale_factor);
				acc[i++] = (int8_t)temp;
				// You now have raw acceleration of gravity in lis3dh->x, y, and z.
			}
		}
		//pass data to step counting algorithm, 4 seconds at a time (which is the WINDOW_LENGTH). put the data into a temporary buffer each loop
		int8_t   data[NUM_TUPLES*3] = {0};
		uint8_t  num_segments       = NUM_SAMPLES/(SAMPLING_RATE*WINDOW_LENGTH);
		uint16_t j                  = 0;

		for(i = 0; i < num_segments; i++) {
			for(j = SAMPLING_RATE*WINDOW_LENGTH*i*3; j < SAMPLING_RATE*WINDOW_LENGTH*(i+1)*3; j++) {
				data[j-SAMPLING_RATE*WINDOW_LENGTH*i*3] = acc[j];
			}
			num_steps += count_steps(data);
		}

		ssd1306_Fill(Black);
		ssd1306_SetCursor(2,0);
		ssd1306_WriteString("Steps:", Font_11x18, White);
		ssd1306_SetCursor(2,15);
		ssd1306_WriteString(itoa(num_steps,message,10), Font_11x18, White);
		ssd1306_SetCursor(2,30);
		ssd1306_WriteString("Distance:", Font_11x18, White);

		if((pre_lat == 0) && (pre_lon == 0)) {
		  ssd1306_SetCursor(2,50);
		  ssd1306_WriteString("Need GPS Lock", Font_7x10, White);
		  ssd1306_UpdateScreen();
		}
		else {
			sprintf(sd1,"%f,%f,%s",pre_lat,pre_lon,pre_time);

			fres = f_mount(&FatFs, "", 1); //1=mount now
			if(fres != FR_OK) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				while(1);
			}

			fres = f_open(&fil, "crds.txt", FA_WRITE | FA_OPEN_ALWAYS);
			if(fres == FR_OK) {
				f_lseek(&fil, f_size(&fil));
				strncpy((char*)writeBuf1, sd1, 33);
				fres = f_write(&fil, writeBuf1, 33, &bytesWrote);
				if(fres != FR_OK) {
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
					while(1);
				}
			}
			else {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				while(1);
			}

			f_close(&fil);

			f_mount(NULL, "", 0);

			new_distance = calculateDistance(pre_lat, pre_lon, cur_lat, cur_lon);
			if (new_distance > MIN_GPS_DISTANCE){
			  total_distance += new_distance;
			}
			if (total_distance < 400)
			{
			  sprintf(buf1,"%0.2f meters",total_distance);
			}
			else{
			  miles = total_distance / METERS_TO_MILES;
			  sprintf(buf1,"%0.2f miles",miles);
			}
			ssd1306_SetCursor(2,45);
			ssd1306_WriteString(buf1, Font_11x18, White);
			ssd1306_UpdateScreen();

		}
			  pre_lat = cur_lat;
			  pre_lon = cur_lon;
			  strcpy(pre_time, cur_time);
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
