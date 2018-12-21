/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "device_led.h"
#include "device_relay.h"
#include "device_photoInterrupter.h"
#include "device_PIRsensor.h"
#include "device_buzzer.h"
#include "device_button.h"
#include "device_ultraSonic.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stateMachine.h"
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef* phuart1;
USER_ACTION* tCurrent_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void EXTI1_IRQHandler_Config(void);
static void EXTI4_IRQHandler_Config(void);
static void EXTI9_5_IRQHandler_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// User defined : Callback Function
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {

	ultraSonic_measureCaptureVal(htim);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	ultraSonic_trigger();

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Setting for UltraSonic function */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
#if 1
#else		// ORIG
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /* Initialize interrupts */
  EXTI1_IRQHandler_Config();
  EXTI4_IRQHandler_Config();
  EXTI9_5_IRQHandler_Config();

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* Setting for UltraSonic function */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

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

#if 0	// OLED TEST
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void EXTI1_IRQHandler_Config(void) {			// Emergency Button
	/*Enable and set EXTI lines 1 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static void EXTI4_IRQHandler_Config(void) {			//exteranl Photo Interrupter
	/*Enable and set EXTI lines 2 to 3 Interrupt*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

static void EXTI9_5_IRQHandler_Config(void) {		// Inner Photo Interrupter // PIR Motion Sensor
	/*Enable and set EXTI lines 4 to 15 Interrupt*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t Action) {

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	printf("______________________CallBack In\n");
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

#if 0		// PIR SENSOR was not adapted in babybox
	if(Action == MOTION_DETECTING) {
		baby_state = BABY_IN;
	}
#endif

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
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  printf("You got error handler");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
