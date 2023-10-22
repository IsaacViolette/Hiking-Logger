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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES_IN_CSV_FILE 400//400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

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
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
UART_HandleTypeDef huart2;
uint8_t nmea;
char nmea_buf[256];
char nmea_gga[256];
uint8_t i = 0;

double cur_lat = 0;
double cur_lon = 0;
double pre_lat = 0;
double pre_lon = 0;
double alt = 0;

double get_lat(char *gga)
{
	double latitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if ((strcmp(token, "N") == 0) || (strcmp(token, "S") == 0))
		{
			break;
		}
		else
		{
			latitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return latitude;
}

double get_lon(char *gga)
{
	double longitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if ((strcmp(token, "W") == 0) || (strcmp(token, "E") == 0))
		{
			break;
		}
		else
		{
			longitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return longitude;
}

double get_alt(char *gga)
{
	double altitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if (strcmp(token, "M") == 0)
		{
			break;
		}
		else
		{
			altitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return altitude;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {

        nmea_buf[i++] = nmea;
        //char buf1[16];
        //char buf2[16];

        if (nmea == '\n' || i >= sizeof(nmea_buf) - 1) {
        	if(nmea_buf[3]=='G' && nmea_buf[4]=='G' && nmea_buf[5] == 'A')
        	{
        		memcpy(nmea_gga, nmea_buf, 256);
        		cur_lat = get_lat(nmea_gga);
        		cur_lon = get_lon(nmea_gga);
        		alt = get_alt(nmea_gga);

        	}

            memset(nmea_buf, 0, sizeof(nmea_buf));
            i = 0;
        }

        // Start the next reception
        HAL_UART_Receive_IT(&huart2, &nmea, 1);
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char buf1[16];
  char buf2[16];

  /* USER CODE END 2 */
  status = lis3dh_init(&lis3dh, &hi2c3, xyz_buf, 6);

          if (status != HAL_OK) {
          	 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
            // Unable to communicate with device!
          }


          char message[64] = "Starting Up";
            //ssd1306_TestAll();
            ssd1306_Init();
            ssd1306_Fill(Black);
            ssd1306_SetCursor(2,0);
            ssd1306_WriteString(message, Font_11x18, White);
            ssd1306_UpdateScreen();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
          uint8_t  num_steps  = 0;
          double distance = 0;
          //char buf[20];
          HAL_UART_Receive_IT(&huart2, &nmea, 1);
          while (1)
                {
          	//hold the data from the CSV file in a fifo-like data structure where the accelerometer data looks like
          	        	    	    //[x1,y1,z1,x2,y2,z2...x400,y400,z400]
          	        	    	    int8_t acc[NUM_SAMPLES_IN_CSV_FILE*3] = {0};
          	        	    	    uint16_t i    = 0;
          	        	    	    float    temp = 0;
          	        	    	    while(i < NUM_SAMPLES_IN_CSV_FILE*3) //while data array is being filled
          	        	        	{
          	        	    			  HAL_Delay(50); //20Hz
          	        	    			  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

          	        	    			//scaling factor to convert the decimal data to int8 integers. calculated in matlab by taking the absolute value of all the data
          	        	    			//and then calculating the max of that data. then divide that by 127 to get the scaling factor
          	        	    			  float scale_factor = 55.3293;

          	        					  if (lis3dh_xyz_available(&lis3dh)) {
          	        							status = lis3dh_get_xyz(&lis3dh);
          	        							float xx = lis3dh.x/16384;
          	        							float yy = lis3dh.y/16384;
          	        							float zz = lis3dh.z/16384;

          	        							temp     = roundf(xx*scale_factor);
          	        							acc[i++] = (int8_t)temp;

          	        							temp     = roundf(yy*scale_factor);
          	        							acc[i++] = (int8_t)temp;

          	        							temp     = roundf(zz*scale_factor);
          	        							acc[i++] = (int8_t)temp;

          	        							//printf("%f, %f, %f\r\n",xx,yy,zz);
          	        							// You now have raw acceleration of gravity in lis3dh->x, y, and z.

          	        						  }
          	        	        	  }
          	        	        	  //pass data to step counting algorithm, 4 seconds at a time (which is the WINDOW_LENGTH). put the data into a temporary buffer each loop
          	        	        	      int8_t   data[NUM_TUPLES*3] = {0};
          	        	        	      uint8_t  num_segments       = NUM_SAMPLES_IN_CSV_FILE/(SAMPLING_RATE*WINDOW_LENGTH);
          	        	        	      uint16_t j                  = 0;

          	        	        	      for (i = 0; i < num_segments; i++) {
          	        	        	          for (j = SAMPLING_RATE*WINDOW_LENGTH*i*3; j < SAMPLING_RATE*WINDOW_LENGTH*(i+1)*3; j++) {
          	        	        	              data[j-SAMPLING_RATE*WINDOW_LENGTH*i*3] = acc[j];
          	        	        	          }
          	        	        	          num_steps += count_steps(data);
          	        	        	      }

          	        	        	      //printf("num steps: %i\n\r", num_steps);
          	        	        	      ssd1306_Fill(Black);
          	        	        	      ssd1306_SetCursor(2,0);
          	        	        	      ssd1306_WriteString("Steps:", Font_11x18, White);
          	        	        	      ssd1306_SetCursor(2,15);
          	        	        	      ssd1306_WriteString(itoa(num_steps,message,10), Font_11x18, White);
          	        	        	      //ssd1306_SetCursor(2,30);
          	        	        	      //ssd1306_WriteString("Distance:", Font_11x18, White);
//          	        	        	      sprintf(buf1,"%0.4f",cur_lat);
//          	        	        	      ssd1306_WriteString(buf1, Font_11x18, White);
//          	        	        	      ssd1306_SetCursor(2,45);
//          	        	        	      ssd1306_WriteString("20 miles", Font_11x18, White);
//          	        	        	      sprintf(buf2,"%0.4f",cur_lon);
//          	        	        	      ssd1306_WriteString(buf2, Font_11x18, White);
          	        	        	      ssd1306_UpdateScreen();

          	        	        	      if((pre_lat == 0) && (pre_lon == 0))
          	        	        	      {
          	        	        	    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
          	        	        	    	pre_lat = cur_lat;
          	        	        	    	pre_lon = cur_lon;
          	        	        	      }
          	        	        	      else
          	        	        	      {
          	        	        	    	distance += calculateDistance(pre_lat, pre_lon, cur_lat, cur_lon);
											sprintf(buf1,"%0.4f",distance);
											ssd1306_SetCursor(2,45);
											ssd1306_WriteString(buf1, Font_11x18, White);
											ssd1306_UpdateScreen();
          	        	        	      }

											pre_lat = cur_lat;
											pre_lon = cur_lon;

      /* USER CODE END WHILE */
                }
      /* USER CODE BEGIN 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

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
