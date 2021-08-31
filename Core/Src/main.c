/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t int_flag = 0;
uint16_t prescal = 8000;
uint8_t UART_buffer[1];
uint16_t UART_STRING[12];
uint16_t UART_Counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Passo1(void);
void Passo2(void);
void Passo3(void);
void Passo4(void);
void motor_direita(uint16_t passo);
void motor_esquerda(uint16_t passo);
void mudar_velocidade_motor(void);
void uart_send_variable(uint16_t value);
void MENU(void);	// <--Implementar menu para alterar angulos minimo, maximo e a velocidade.
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
	uint16_t adc_angulo;
	uint16_t min_ang = 200;
	uint16_t max_ang = 3600;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWM_GPIO_Port, PWM_Pin, GPIO_PIN_SET);
  NVIC_SetPriority(USART1_IRQn, 1);// prioridade maior
  HAL_ADC_Start(&hadc);
  HAL_UART_Transmit(&huart1, (uint8_t *)"ADC STARTED\r\n", sizeof("ADC STARTED\r\n"), 100);
  HAL_TIM_Base_Start_IT(&htim3);
  NVIC_SetPriority(TIM3_IRQn, 3); //prioridade menor
  HAL_UART_Transmit(&huart1, (uint8_t *)"Timer STARTED\r\n", sizeof("Timer STARTED\r\n"), 100);
  HAL_UART_Receive_IT(&huart1, UART_buffer, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)"UART receive interrupt STARTED\r\n", sizeof("UART receive interrupt STARTED\r\n"), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"speed: ", sizeof("speed: "), 100);
  uart_send_variable((int)prescal);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", sizeof("\r\n"), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // lê o adc
  adc_angulo = HAL_ADC_GetValue(&hadc);

  //se desloca para a posição de angulo minimo
  if (adc_angulo >= min_ang){
	do{
		motor_esquerda(int_flag);
		adc_angulo = HAL_ADC_GetValue(&hadc);
	}while(adc_angulo >= min_ang);
  }
  while (1)
  {
	//se desloca do angulo minimo para o angulo maximo
	if (adc_angulo <= min_ang){
	  do{
		  motor_direita(int_flag);
		  adc_angulo = HAL_ADC_GetValue(&hadc);
	  }while(adc_angulo <= max_ang);
	}
	//se desloca do angulo minimo para o angulo maximo
	else if(adc_angulo >= max_ang){
	  do{
		  motor_esquerda(int_flag);
		  adc_angulo = HAL_ADC_GetValue(&hadc);
	  }while(adc_angulo >= min_ang);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  HAL_ADC_Stop(&hadc);
  HAL_TIM_Base_Stop_IT(&htim3);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|M1B_Pin
                          |M1A_Pin|M2B_Pin|M2A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA8 PA9 PA10
                           PA11 PA12 PWM_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB13 PB14 PB15 M1B_Pin
                           M1A_Pin M2B_Pin M2A_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|M1B_Pin
                          |M1A_Pin|M2B_Pin|M2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Passo1(void){
	HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_SET);
}

void Passo2(void){
	HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_SET);
}

void Passo3(void){
	HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_RESET);
}

void Passo4(void){
	HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_RESET);
}

//função para fazer o motor ir para a esquerda
void motor_esquerda(uint16_t passo){
	if (passo == 0){
		Passo1();
	}
	else if (passo == 1){
		Passo2();
	}
	else if(passo == 2){
		Passo3();
	}
	else if (passo == 3){
		Passo4();
	}
}

//função para fazer o motor ir para a direita
void motor_direita(uint16_t passo){
	if (passo == 0){
		Passo4();
	}
	else if (passo == 1){
		Passo3();
	}
	else if(passo == 2){
		Passo2();
	}
	else if(passo == 3){
		Passo1();
	}
}

//função para mudar a velocidade do motor
void mudar_velocidade_motor(void){
	int16_t temp = 0;
	uint16_t temp_mult = 1;
	uint32_t UART_Value = 0;
	for (temp = UART_Counter -2; temp >= 0; temp--){
		UART_Value = UART_Value + UART_STRING[temp] * temp_mult;
		temp_mult = temp_mult*10;
	}
	prescal = UART_Value;
	__HAL_TIM_SET_PRESCALER(&htim3,prescal); // ajustar prescaller do timer para alterar a velocidade
	UART_Counter = 0;
	HAL_UART_Transmit(&huart1, (uint8_t *)"speed: ", sizeof("speed: "), 100);
	uart_send_variable((int)prescal);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", sizeof("\r\n"), 100);
}

//função enviar variavel pela uart
void uart_send_variable(uint16_t value){ //consertar: adicionando digitos incoerentes ao final ex:9500$
	char temp_buffer[12];
	uint16_t i = 0, length = 0;
	itoa(value, temp_buffer, 10);
	//for como alternativa a sizeof(algum problema na contagem com sizeof)
	for (i = 0; temp_buffer[i] != '\0' && temp_buffer[i] != '\n'; i++) {
	    length++;
	  }
	HAL_UART_Transmit(&huart1, (uint8_t *)temp_buffer, length, 100);
}

// timer interrupt = controle da velocidade do motor
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	int_flag++;
	if (int_flag > 3){
		int_flag = 0;
	}
}

// UART receive interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(&huart1, UART_buffer, sizeof(UART_buffer), 100);
	UART_STRING[UART_Counter] = atoi((const char *)UART_buffer);
	UART_Counter++;
	if (UART_buffer[0] == '\r'){
		HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", sizeof("\r\n"), 100);
		mudar_velocidade_motor();

	}
    HAL_UART_Receive_IT(&huart1, UART_buffer, 1);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
