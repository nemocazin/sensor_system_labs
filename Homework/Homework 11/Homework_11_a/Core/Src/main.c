/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCLK_PIN GPIOB, GPIO_PIN_6

#define PIN_C0 GPIOC, GPIO_PIN_11
#define PIN_C1 GPIOC, GPIO_PIN_10
#define PIN_C2 GPIOC, GPIO_PIN_9
#define PIN_C3 GPIOC, GPIO_PIN_8

#define PIN_R0 GPIOC, GPIO_PIN_3
#define PIN_R1 GPIOC, GPIO_PIN_2
#define PIN_R2 GPIOC, GPIO_PIN_13
#define PIN_R3 GPIOC, GPIO_PIN_12

#define DEBOUNCE_TIME 2

#define GREEN_LED_PIN GPIOA, GPIO_PIN_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int column_index_rec = 0;

uint8_t matrix[5][2] = {
		{0, 16},  // y0 content {row}, address {column}
		{0, 8},   // y1
		{0, 4},   // y2
		{0, 2},   // y3
		{0, 1}
};

uint8_t matrix_QM[5][2] = {
		{32, 16},  // y0 content {row}, address {column}
		{64, 8},   // y1
		{69, 4},   // y2
		{72, 2},   // y3
		{48, 1}
};

uint8_t matrix_A[5][2] = {
		{31, 16},  // y0 content {row}, address {column}
		{36, 8},   // y1
		{68, 4},   // y2
		{36, 2},   // y3
		{31, 1}
};

uint8_t matrix_B[5][2] = {
		{127, 16},  // y0 content {row}, address {column}
		{73, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{54, 1}
};

uint8_t matrix_C[5][2] = {
		{62, 16},  // y0 content {row}, address {column}
		{65, 8},   // y1
		{65, 4},   // y2
		{65, 2},   // y3
		{34, 1}
};

uint8_t matrix_D[5][2] = {
		{127, 16},  // y0 content {row}, address {column}
		{65, 8},   // y1
		{65, 4},   // y2
		{65, 2},   // y3
		{62, 1}
};

uint8_t matrix_E[5][2] = {
		{127, 16},  // y0 content {row}, address {column}
		{73, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{73, 1}
};

uint8_t matrix_F[5][2] = {
		{127, 16},  // y0 content {row}, address {column}
		{72, 8},   // y1
		{72, 4},   // y2
		{72, 2},   // y3
		{72, 1}
};

uint8_t matrix_0[5][2] = {
		{62, 16},  // y0 content {row}, address {column}
		{113, 8},   // y1
		{73, 4},   // y2
		{71, 2},   // y3
		{62, 1}
};

uint8_t matrix_1[5][2] = {
		{16, 16},  // y0 content {row}, address {column}
		{33, 8},   // y1
		{127, 4},   // y2
		{1, 2},   // y3
		{0, 1}
};

uint8_t matrix_2[5][2] = {
		{33, 16},  // y0 content {row}, address {column}
		{67, 8},   // y1
		{69, 4},   // y2
		{73, 2},   // y3
		{49, 1}
};

uint8_t matrix_3[5][2] = {
		{34, 16},  // y0 content {row}, address {column}
		{65, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{54, 1}
};

uint8_t matrix_4[5][2] = {
		{120, 16},  // y0 content {row}, address {column}
		{8, 8},   // y1
		{8, 4},   // y2
		{8, 2},   // y3
		{127, 1}
};

uint8_t matrix_5[5][2] = {
		{114, 16},  // y0 content {row}, address {column}
		{81, 8},   // y1
		{81, 4},   // y2
		{81, 2},   // y3
		{78, 1}
};

uint8_t matrix_6[5][2] = {
		{62, 16},  // y0 content {row}, address {column}
		{73, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{38, 1}
};

uint8_t matrix_7[5][2] = {
		{64, 16},  // y0 content {row}, address {column}
		{64, 8},   // y1
		{79, 4},   // y2
		{80, 2},   // y3
		{96, 1}
};

uint8_t matrix_8[5][2] = {
		{54, 16},  // y0 content {row}, address {column}
		{73, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{54, 1}
};

uint8_t matrix_9[5][2] = {
		{50, 16},  // y0 content {row}, address {column}
		{73, 8},   // y1
		{73, 4},   // y2
		{73, 2},   // y3
		{62, 1}
};

char map[16] = "FB73EA62D951C840"; // Keyboard characters definition

uint32_t keypress[16];	// Array to keep track of
uint32_t ack[16];		// Array for key press acknowledge

int column_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char RX_byte = 0;
char command = 0;
int new_command = 0;

int BaudElapsedFlag = 0;

char RX_byte = 0;
char RX_byte2 = 0;

char UART_RX_byte;
char UART_RX_flag;

char string [32];

uint8_t CharMAP[256][5][2] = {
		{32, 16},  // y0 content {row}, address {column}
		{64, 8},   // y1
		{69, 4},   // y2
		{72, 2},   // y3
		{48, 1}
};

void initCharMAP (void)
{
	for (int i = 0; i < 256; i++)
	{
		memcpy(CharMAP[i], matrix_QM, sizeof(matrix_QM));
	}
	memcpy(CharMAP[(uint8_t) '0'], matrix_0, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '1'], matrix_1, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '2'], matrix_2, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '3'], matrix_3, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '4'], matrix_4, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '5'], matrix_5, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '6'], matrix_6, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '7'], matrix_7, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '8'], matrix_8, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '9'], matrix_9, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'A'], matrix_A, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'B'], matrix_B, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'C'], matrix_C, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'D'], matrix_D, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'E'], matrix_E, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) 'F'], matrix_F, sizeof(matrix_0));
}

void showLetter (char digit)
{
	memcpy(matrix, CharMAP[(uint8_t) digit], sizeof(matrix));
}

void UART_IR_sendByte (char byte)
{
	// Enable baud rate interrupt timer
	HAL_TIM_Base_Start_IT(&htim10);
	// Send start bit ('0')
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	BaudElapsedFlag = 0;
	while (BaudElapsedFlag == 0)
	{
		// Just wait
	}

	for (int bit_ctr = 0; bit_ctr < 8; bit_ctr++)
	{
		// The Flag has been reset in the periodElapsedCallback of TIM10
		if((byte & (0x01<<bit_ctr)) == 0)
		{
			// We want to send a zero
			// So we have to enable the PWM
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		}
		else
		{
			// We want to send a '1'
			// So we stop the PWM
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		}
		// Then we reset the flag
		BaudElapsedFlag = 0;
		while (BaudElapsedFlag == 0)
		{
			// Just wait
		}
	}

	// We need to send the STOP bit '1'
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	while (BaudElapsedFlag == 0)
	{
		// Just wait
	}
	HAL_TIM_Base_Stop_IT(&htim10);
	BaudElapsedFlag = 0;
}

void UART_IR_Send(char *payload, int size)
{
	for (int i = 0; i < size; i++)
	{
		UART_IR_sendByte(payload[i]);
		HAL_Delay(1);
	}
}



/**
 * @brief Timers Callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIMER 10
	if (htim == &htim10)
	{
		BaudElapsedFlag = 1;
	}

	// TIMER 11
	if (htim == &htim11)
	{
		// Read row pins - remember : active low !
		// If pin read high -> key not pressed -> set counter to zero
		// else -> increase counter to keep track of how long a key is pressed

		if (HAL_GPIO_ReadPin(PIN_R0) == GPIO_PIN_SET){
			keypress[4 * column_index] = 0;
		} else {
			keypress[4 * column_index]++;
		}

		if (HAL_GPIO_ReadPin(PIN_R1) == GPIO_PIN_SET){
			keypress[4 * column_index+1] = 0;
		} else {
			keypress[4 * column_index+1]++;
		}

		if (HAL_GPIO_ReadPin(PIN_R2) == GPIO_PIN_SET){
			keypress[4 * column_index+2] = 0;
		} else {
			keypress[4 * column_index+2]++;
		}

		if (HAL_GPIO_ReadPin(PIN_R3) == GPIO_PIN_SET){
			keypress[4 * column_index+3] = 0;
		} else {
			keypress[4 * column_index+3]++;
		}

		column_index = (++column_index) % 4;

		HAL_GPIO_WritePin(PIN_C0, (column_index==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN_C1, (column_index==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN_C2, (column_index==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN_C3, (column_index==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
	}

	if (htim == &htim5)
	{
		HAL_GPIO_WritePin(RCLK_PIN, GPIO_PIN_SET);
		if(++column_index_rec > 4)
		{
			column_index_rec = 0;
		}
		HAL_GPIO_WritePin(RCLK_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(&hspi1, matrix(column_matrix_rec), 2);
	}
}



/**
 * @brief Callback of UART
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		HAL_UART_Receive_IT(&huart1, &RX_byte, 1);
		command = RX_byte;
		new_command = 1;
	}

	if(huart == &huart2)
	{
		HAL_UART_Receive_IT(&huart2, &RX_byte2, 1);
		command = RX_byte2;
		new_command = 1;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim5);

  initCharMAP();

  HAL_UART_Receive_IT(&huart1, &RX_byte, 1);

  HAL_UART_Receive_IT(&huart2, &RX_byte2, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(new_command)
	  {
		  showLetter(command);
		  new_command = 0;
	  }

	  for (int i = 0; i < 16; i++)
	  {
		  if (keypress[i] > DEBOUNCE_TIME)
		  {
			  // We wait for a minimum duration of keypress to avoid bouncing
			  if(ack[i] == 0)
			  {
				  // if we have yet to acknowledge this event
				  // that means it is a new valid keypress
				  UART_IR_sendByte(map[i]);
				  // Then we ack this keypress so later we ignore it
				  // if it remains pressed
				  ack[i] = 1;
;			  }
		  }
		  else
		  {
			  // The key is released => we remove the ack
			  ack[i] = 0;
		  }
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2210;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 5000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC2 PC3 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
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
