/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
typedef enum {READY = 0, WAIT_RELEASE, COUNTDOWN_DELAY, TIMING} app_state_t;
volatile app_state_t g_state = READY;
volatile uint8_t g_button_event = 0; //set by EXTI
volatile uint32_t reaction_time = 0;
volatile uint8_t g_meas_done = 0;
volatile uint8_t press_seen = 0; // Falling edge
volatile uint8_t start_req = 0; // Full click detected
volatile uint8_t stop_req = 0; // Press during timing
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_seg(GPIO_TypeDef * port, uint16_t pin, GPIO_PinState state) // takes in what port =, pin, and on or off
{
	HAL_GPIO_WritePin(port, pin, state);
}
void disp(uint8_t pattern) // Provided with a hex value for pattern
{
	set_seg(LED_A_GPIO_Port, LED_A_Pin, (pattern & (1<<0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_B_GPIO_Port, LED_B_Pin, (pattern & (1<<1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_C_GPIO_Port, LED_C_Pin, (pattern & (1<<2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_D_GPIO_Port, LED_D_Pin, (pattern & (1<<3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_E_GPIO_Port, LED_E_Pin, (pattern & (1<<4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_F_GPIO_Port, LED_F_Pin, (pattern & (1<<5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	set_seg(LED_G_GPIO_Port, LED_G_Pin, (pattern & (1<<6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	//set_seg(LED_DP_GPIO_Port, LED_DP_Pin, (pattern & (1<<7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if(g_state == TIMING)
		{
			reaction_time++; // Increment by 1ms
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		// Flag for button edge event
		GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if(level == GPIO_PIN_RESET)
		{
			// Button is on falling edge
			if (g_state == READY)
			{
				press_seen = 1; //button on
			}
			else if (g_state == TIMING)
			{
				stop_req = 1; //as soon as state changes, request to stop
			}
		}
		else
		{
			// Button is on rsigin edge
			if(g_state == READY && press_seen)
			{
				press_seen = 0; //reset
				start_req = 1; // full press happened
			}
		}
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
	uint8_t MSG[35];
	unsigned char lookup[16] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};

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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);
  srand(HAL_GetTick()); // Seed random function
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  disp(0x00);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  sprintf(MSG, "Ready\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
  g_state = READY;
  while (1)
  {
	  // Waiting mode
	  if(g_state == READY)
	  {
		  // Wait for interrupt flag
		  if(start_req)
		  {
			  start_req = 0;
			  sprintf(MSG, "Measuring\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
			  g_state = COUNTDOWN_DELAY;
		  }
	  }
	  else if(g_state == COUNTDOWN_DELAY)
	  {
		  disp(lookup[3]);
		  HAL_Delay(1000);
		  disp(lookup[2]);
		  HAL_Delay(1000);
		  disp(lookup[1]);
		  HAL_Delay(1000);
		  disp(0x00); // Clear display
		  HAL_Delay(1000);
		  uint32_t random_time = 1000u + (rand() % 3001u);
		  HAL_Delay(random_time);

		  // Start timing
		  reaction_time = 0;
		  g_meas_done = 0;
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Turn on green LED
		  HAL_TIM_Base_Start_IT(&htim3); // Start timer
		  g_state = TIMING;	// set state to timing state
	  }
	  else if (g_state == TIMING)
	  {
		  if(stop_req)
		  {
			  stop_req = 0;
			  // Stop the timer
			  HAL_TIM_Base_Stop_IT(&htim3);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Turn off
			  uint32_t ms = reaction_time;
			  uint32_t sec = ms / 1000u; // get whole number of seconds
			  uint32_t rem = ms % 1000u; // get remaining in ms

			  sprintf(MSG, "Reaction Time: %lu.%03lu\r\n", sec, rem);
			  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
			  disp(0x00); // Clear display

			  sprintf(MSG, "Ready\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)MSG, strlen(MSG), 100);
			  g_state = READY; //Go back to state 1

		  }
	  }

	  // Display reaction time

	  // Go back to Ready mode


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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_A_Pin|LED_B_Pin|LED_F_Pin
                          |LED_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_C_Pin|LED_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Toggle_SW_Pin */
  GPIO_InitStruct.Pin = Toggle_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Toggle_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_A_Pin LED_B_Pin LED_F_Pin
                           LED_E_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_A_Pin|LED_B_Pin|LED_F_Pin
                          |LED_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_C_Pin LED_DP_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_C_Pin|LED_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_D_Pin */
  GPIO_InitStruct.Pin = LED_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_D_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(200);
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
