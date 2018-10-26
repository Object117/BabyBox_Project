
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "device_led.h"
#include "device_relay.h"
#include "device_photoInterrupter.h"
#include "device_PIRsensor.h"
#include "device_buzzer.h"
#include "device_button.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stateMachine.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef* phuart1;

USER_ACTION* tCurrent_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void EXTI1_IRQHandler_Config(void);
static void EXTI4_IRQHandler_Config(void);
static void EXTI9_5_IRQHandler_Config(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
#if 0
  MX_GPIO_Init();
#else
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  EXTI1_IRQHandler_Config();
  EXTI4_IRQHandler_Config();
  EXTI9_5_IRQHandler_Config();

  LED_Init(RED_LED, GREEN_LED, BLUE_LED);
  //  PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  PIS_Init(PIS1);
  PIS_Init(PIS2);
  PIR_Sensor_Init(PIR_SENSOR1);
  Relay_Init(RELAY1);
  Relay_Init(RELAY2);
  //BUZZER_Init(BUZZER_INTERNAL);
  Buzzer_Init(BUZZER_EXTERNAL);
  Button_Init(EMERGENCY_BUTTON);

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  extdoor_status = EXT_DOOR_CLOSE;
  innerdoor_state = INNER_DOOR_CLOSE;
  baby_state = BABY_NONE;

  changeingState = initialize_state();

// uint8_t Welcome[] = "Welcome";
// HAL_UART_Transmit(&huart1, (uint8_t*)Welcome, sizeof(Welcome), 10);		// TESTING
  printf("Welcome!!");

#if 1	// OLED TEST
  ssd1306_Init();

  HAL_Delay(1000);
  ssd1306_Fill(White);
  ssd1306_UpdateScreen();

  HAL_Delay(50);

  ssd1306_SetCursor(23,23);
  ssd1306_WriteString("Hello World",Font_11x18,Black);
  ssd1306_UpdateScreen();

  HAL_Delay(2000);
  ssd1306_Fill(White);		// For Clear
  ssd1306_UpdateScreen();

  ssd1306_SetCursor(5,23);
  ssd1306_WriteString("World Wide",Font_11x18,Black);
  ssd1306_UpdateScreen();

#endif

  while(1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  tCurrent_state = change_state();
	  tCurrent_state->extdoor_open();
	  tCurrent_state->extdoor_close();
	  tCurrent_state->inner_door_open();
	  tCurrent_state->inner_door_close();
	  tCurrent_state->baby_in();
	  tCurrent_state->baby_none();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LoadCell_CLK_Pin|Buzzer_CTRL_Pin|ELECTROMAGNET_CTRL_1_Pin|ELECTROMAGNET_CTRL_2_Pin 
                          |BAR_LED_CTRL_1_Pin|LED_G_EXTERNAL_Pin|LED_B_EXTERNAL_Pin|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BAR_LED_CTRL_2_Pin|USER_LED_STATUS_Pin|LED_R_EXTERNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_CTRL_EXTRNAL_GPIO_Port, Buzzer_CTRL_EXTRNAL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PHOTO_1_EXTI4_Pin PHOTO_2_EXTI5_Pin PIR_EXTI6_Pin USER_KEY_1_EXTI_Pin */
  GPIO_InitStruct.Pin = PHOTO_1_EXTI4_Pin|PHOTO_2_EXTI5_Pin|PIR_EXTI6_Pin|USER_KEY_1_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LoadCell_DT_Pin */
  GPIO_InitStruct.Pin = LoadCell_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LoadCell_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LoadCell_CLK_Pin Buzzer_CTRL_Pin ELECTROMAGNET_CTRL_1_Pin ELECTROMAGNET_CTRL_2_Pin 
                           BAR_LED_CTRL_1_Pin LED_G_EXTERNAL_Pin LED_B_EXTERNAL_Pin Buzzer_CTRL_EXTRNAL_Pin 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = LoadCell_CLK_Pin|Buzzer_CTRL_Pin|ELECTROMAGNET_CTRL_1_Pin|ELECTROMAGNET_CTRL_2_Pin 
                          |BAR_LED_CTRL_1_Pin|LED_G_EXTERNAL_Pin|LED_B_EXTERNAL_Pin|Buzzer_CTRL_EXTRNAL_Pin 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergence_BTN_EXTI1_Pin */
  GPIO_InitStruct.Pin = Emergence_BTN_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Emergence_BTN_EXTI1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BAR_LED_CTRL_2_Pin USER_LED_STATUS_Pin LED_R_EXTERNAL_Pin */
  GPIO_InitStruct.Pin = BAR_LED_CTRL_2_Pin|USER_LED_STATUS_Pin|LED_R_EXTERNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void EXTI1_IRQHandler_Config(void) {			// Emergency Button
	/*Enable and set EXTI lines 1 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static void EXTI4_IRQHandler_Config(void) {			//exteranl Photo Interrupter
	/*Enable and set EXTI lines 2 to 3 Interrupt*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

static void EXTI9_5_IRQHandler_Config(void) {		// Inner Photo Interrupter // PIR Motion Sensor
	/*Enable and set EXTI lines 4 to 15 Interrupt*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t Action) {

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);

	if(Action == EXT_DOOR_PIN) {
		switch(HAL_GPIO_ReadPin(EXT_DOOR_PORT, EXT_DOOR_PIN)) {
			case GPIO_PIN_SET	:
				extdoor_status = EXT_DOOR_CLOSE;
				break;

			case GPIO_PIN_RESET	:
				extdoor_status = EXT_DOOR_OPEN;
				break;

			default		:	// exception
				extdoor_status = EXT_DOOR_CLOSE;
				break;
		}
	}

	if(Action == INNER_DOOR_PIN) {
		switch(HAL_GPIO_ReadPin(INNER_DOOR_PORT, INNER_DOOR_PIN)) {
			case GPIO_PIN_SET	:
				innerdoor_state = INNER_DOOR_CLOSE;
				break;

			case GPIO_PIN_RESET	:
				innerdoor_state = INNER_DOOR_OPEN;
				break;

			default		:	// exception
				innerdoor_state = INNER_DOOR_CLOSE;
				break;
		}
	}

	if(Action == MOTION_DETECTING) {
		baby_state = BABY_IN;
	}

	if(Action == BUTTON_EMERGENCY) {
		switch(HAL_GPIO_ReadPin(BUTTON_1_PORT, BUTTON_EMERGENCY)) {
			case GPIO_PIN_SET	:
				changeingState = recovery_state();
				break;

			case GPIO_PIN_RESET	:
				break;

			default		:	// exception
				break;
		}
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
