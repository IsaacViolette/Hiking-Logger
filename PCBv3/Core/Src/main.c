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
#include <stdarg.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

HAL_StatusTypeDef status;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* User Global Variables -----------------------------------------------------*/
lis3dh_t lis3dh;

/* Longitude and Latitude variables used for reading and writing */
double cur_lat = 0;
double cur_lon = 0;
double pre_lat = 0;
double pre_lon = 0;

/* Time is 9 characters long, so 9 bytes */
char cur_time[9];
char pre_time[9];

/* Reads altitude variable, not currently used */
double alt = 0;

/* GPS buffers */
char nmea_buf[128]; //This is used for every NMEA sentence
char nmea_gga[128]; //This stores only GGA sentences
uint8_t nmea; //Holds integer character for every interrupt on UART1
uint8_t i = 0; //Index for NMEA sentences

/*
 * This is the callback function that gets called by the HAL
 * library when a UART receive operation is complete. Once interrupt
 * occurs, this function is called. It stores ASCII (integer representation)
 * into buffer. Once GGA sentence is found, latitude, longitude and time are
 * stored to global variables. Start another interrupt reception
 * once function is complete.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {

        nmea_buf[i++] = nmea; //store character

        //check if sentence is complete or if buffer overflow occurs
        if (nmea == '\n' || i >= sizeof(nmea_buf) - 1) {
        	if(nmea_buf[3]=='G' && nmea_buf[4]=='G' && nmea_buf[5] == 'A') //check for GGA sentence
        	{
        		memcpy(nmea_gga, nmea_buf, 128); //copy general buffer to GGA buffer
        		cur_lat = get_lat(nmea_gga);
        		cur_lon = get_lon(nmea_gga);
        		get_time(nmea_gga, pre_time);
        		alt = get_alt(nmea_gga);
        	}

            memset(nmea_buf, 0, sizeof(nmea_buf)); //clear general buffer
            i = 0; //reset index
        }

        // Start next UART character reception
        HAL_UART_Receive_IT(&huart1, &nmea, 1);
        if(status != HAL_OK)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			while(1);
		}
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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

	// Allocate a buffer for reading data from the sensor.
	// Two bytes required to read X accelerometer data.
	uint8_t x_accel_buf[2] = { 0 };
	signed int x_accel; //raw x acceleration

	/* Buffer to write variables to display */
	char display_buf[16];

	/* Variables for state machine */
	uint8_t state = BELOW; //arbitrary starting point
	uint16_t steps = 0; //total steps

	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	UINT bytesWrote;
	FRESULT fres;
	BYTE writedisplay_buf[35];
	char sd1[35]; //buffer for SD write, 35 characters always

	float total_distance = 0;
	float new_distance;
	float miles;

	HAL_Delay(1000); //Allow for microSD to settle (initialize everything)

	/* Create new file on MicroSD */
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if(fres != FR_OK) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		while(1);
	}
	fres = f_open(&fil, "crds.txt", FA_CREATE_ALWAYS | FA_OPEN_ALWAYS);
	if(fres != FR_OK) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		while(1);
	}
	fres = f_close(&fil);
	if(fres != FR_OK) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		while(1);
	}
	fres = f_mount(NULL, "", 0); //0=demount
	if(fres != FR_OK) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		while(1);
	}

	/* Initialize accelerometer */
	status = lis3dh_init(&lis3dh, &hi2c1, x_accel_buf, 2);
	if (status != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		while(1);
	}

	/* Preliminary display of Steps and distance */
	ssd1306_Init();
	ssd1306_SetCursor(2,0);
	ssd1306_WriteString("Steps:", Font_11x18, White);
	ssd1306_SetCursor(2,20);
	ssd1306_WriteString("Distance:", Font_11x18, White);
	ssd1306_UpdateScreen();

	/* Start character interrupt interrupt */
	status = HAL_UART_Receive_IT(&huart1, &nmea, 1);
	if(status != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		while(1);
	}

	while (1)
	{
		/* Waits predetermined number of samples, then updates screen */
		for(int i = 0; i < NUM_SAMPLES; i++)
		{
			HAL_Delay(50); //20Hz sampling rate

			/* State machine  */
			if(lis3dh_xyz_available(&lis3dh)) {
				status = lis3dh_get_xyz(&lis3dh);
				if(status != HAL_OK)
				{
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
					while(1);
				}

				x_accel = lis3dh.x; //get x acceleration

				switch(state) {
					case TOP: //if in TOP state
						if(x_accel < THRESH_LOW) { //if acceleration drops below threshold
							steps += 1; //add one step
							state = BELOW; //change state
						}
						break;
					case BELOW: //if in BOTTOM state
						if(x_accel > THRESH_HIGH) { //if acceleration surpasses threshold
							state = TOP; //change states
						}
						break;
				}
			}
		}

		/* Write Steps to display */
		ssd1306_SetCursor(70,0);
		ssd1306_WriteString(itoa(steps,display_buf,10), Font_11x18, White);

		/* Chacks for GPS lock, lat and long will be zero in this case*/
		if((pre_lat == 0) && (pre_lon == 0)) {
		  ssd1306_SetCursor(2,50);
		  ssd1306_WriteString("Need GPS Lock", Font_7x10, White);
		  ssd1306_UpdateScreen();
		}
		else {
			sprintf(sd1,"%fN,%fW,%s",pre_lat,pre_lon,pre_time); //write lat,long,time to sd card buffer

			fres = f_mount(&FatFs, "", 1); //1=mount now
			if(fres != FR_OK) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				while(1);
			}

			fres = f_open(&fil, "crds.txt", FA_WRITE | FA_OPEN_ALWAYS);
			if(fres == FR_OK) {
				f_lseek(&fil, f_size(&fil)); //seek through file to find next line
				strncpy((char*)writedisplay_buf, sd1, 35); //copy general string to required sd card buffer
				fres = f_write(&fil, writedisplay_buf, 35, &bytesWrote);
				if(fres != FR_OK) {
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
					while(1);
				}
			}
			else {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				while(1);
			}

			f_close(&fil); //close file

			f_mount(NULL, "", 0); //dismout file

			new_distance = calculateDistance(pre_lat, pre_lon, cur_lat, cur_lon);
			if(new_distance > MIN_GPS_DISTANCE){ //prevent GPS drift when still
			  total_distance += new_distance;
			}
			if(total_distance < 400) //display meters
			{
			  sprintf(display_buf,"%0.2f m     ",total_distance);
			}
			else{ //change from meters to miles
			  miles = total_distance / METERS_TO_MILES;
			  sprintf(display_buf,"%0.2f Mi     ",miles);
			}
			ssd1306_SetCursor(2,40);
			ssd1306_WriteString(display_buf, Font_11x18, White);
			ssd1306_UpdateScreen();

		}
			/* update present values to past */
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
