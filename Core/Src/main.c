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
#include "liquidcrystal_i2c.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LD2_PIN GPIO_PIN_5
#define LD2_PORT GPIOA

int ADC_READ(uint32_t ADC_CHANNEL) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 1);
    return HAL_ADC_GetValue(&hadc1);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  char printout[16];

  long refreshRate = 2;
  uint32_t lastRefreshTick = HAL_GetTick();
  uint32_t now;
  HD44780_Init(2);

  uint16_t joystick_val[3];
  char cursor_x = 0;
  _Bool cursor_y = 1;
  _Bool went_right = 0;
  _Bool went_left = 0;
  _Bool went_y = 0;
  _Bool pressed = 0;

  char time[3] = {0};
  char time_limit[3] = {24, 60,60};
  char* time_print[] = {"hr ", "min", "sec"};
  char time_state = 0;
  unsigned short adding[] = {1, 5, 10, 20};
  unsigned int time_remaining;
  uint32_t start_time;
  unsigned int total_time;

  int remaining_pause;

  _Bool servo_state = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  enum Page screen = SELECTING;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // Start conversion for Channel 6 (PA0)

      now = HAL_GetTick();

      joystick_val[0] = ADC_READ(ADC_CHANNEL_6);//x_axis
      joystick_val[1] = ADC_READ(ADC_CHANNEL_7);//y_axis
      joystick_val[2] = ADC_READ(ADC_CHANNEL_8);//Switch button

      if(screen == SELECTING) {
          if (!went_right && joystick_val[0] > 4000) {
              went_right = 1;
          } else if (went_right && joystick_val[0] < 4000) {
              if (cursor_y)
                  cursor_x = (cursor_x + 4) % 16;
              else
                  cursor_x = 12;
              went_right = 0;
          }
          if (!went_left && joystick_val[0] < 20) {
              went_left = 1;
          } else if (went_left && joystick_val[0] > 20) {
              if (cursor_y)
                  cursor_x = (cursor_x - 4) % 16;
              else
                  cursor_x = 8;
              went_left = 0;
          }


          if (pressed == 0 && joystick_val[2] < 20) {
              pressed = 1;
              if (cursor_y) {
                  time[time_state] = (time[time_state] + adding[cursor_x / 4]) % time_limit[time_state];
              }
              else {
                  if (cursor_x == 8) {
                      time_state++;
                      if(time_state > 2){
                          time_state = 2;
                          screen = CONFIRMING;
                          cursor_x = 1;
                      }
                  } else {
                      time_state--;
                      time_state = time_state < 0 ? 0 : time_state;
                  }
              }
          } else if (pressed && joystick_val[2] > 20) {
              pressed = 0;
          }


          if (!went_y && joystick_val[1] > 4000) {
              went_y = 1;
          } else if (went_y && joystick_val[1] < 4000) {
              cursor_y = 0;
              cursor_x = 8;
              went_y = 0;
          }
          if (!went_y && joystick_val[1] < 20) {
              went_y = 1;
          } else if (went_y && joystick_val[1] > 20) {
              cursor_y = 1;
              cursor_x = 0;
              went_y = 0;
          }
      }
      else if(screen == CONFIRMING){
          if (!went_right && joystick_val[0] > 4000) {
              went_right = 1;
          } else if (went_right && joystick_val[0] < 4000) {
              cursor_x = 9;
              went_right = 0;
          }
          if (!went_left && joystick_val[0] < 20) {
              went_left = 1;
          } else if (went_left && joystick_val[0] > 20) {
              cursor_x = 1;
              went_left = 0;
          }

          if (pressed == 0 && joystick_val[2] < 20) {
              pressed = 1;
              if(cursor_x == 9){
                  screen = SELECTING;
                  cursor_x = 0;
                  cursor_y = 1;
              }
              else if (cursor_x == 1){
                  screen = COUNTING;
                  start_time = now;
                  total_time = (time[0]*3600 + time[1]*60 + time[2])*1000;
                  servo_state = 1;
                  remaining_pause = 2;
                  cursor_x = 6;
              }
          } else if (pressed && joystick_val[2] > 20) {
              pressed = 0;
          }
      }
      else if(screen == COUNTING){
            if(now-start_time > total_time){
                screen = REMINDING;
                servo_state = 0;
            }
            time_remaining = total_time - (now - start_time);

            if (!went_right && joystick_val[0] > 4000) {
              went_right = 1;
            } else if (went_right && joystick_val[0] < 4000) {
              cursor_x = 7;
              went_right = 0;
            }
            if (!went_left && joystick_val[0] < 20) {
              went_left = 1;
            } else if (went_left && joystick_val[0] > 20) {
              cursor_x = 6;
              went_left = 0;
            }

            if (pressed == 0 && joystick_val[2] < 20) {
              pressed = 1;
              if(cursor_x == 7 && remaining_pause > 0){
                  screen = PAUSING;
                  servo_state = 0;
                  remaining_pause--;
              }
            } else if (pressed && joystick_val[2] > 20) {
              pressed = 0;
            }
      }
      else if(screen == PAUSING){
          if (pressed == 0 && joystick_val[2] < 20) {
              pressed = 1;
              screen = COUNTING;
              servo_state = 1;
              total_time = time_remaining;
              start_time = now;
              cursor_x = 6;
          } else if (pressed && joystick_val[2] > 20) {
              pressed = 0;
          }
      }
      else{
          if (pressed == 0 && joystick_val[2] < 20) {
              pressed = 1;
              screen = SELECTING;
              cursor_x = 0;
              cursor_y = 1;
              time[0] = time[1] = time[2] = 0;
              time_state = 0;
          } else if (pressed && joystick_val[2] > 20) {
              pressed = 0;
          }
      }
//
      if(now - lastRefreshTick >= 1000 / refreshRate){
          HD44780_Clear();
          if(screen == SELECTING) {
              HD44780_SetCursor(0, 1);
              HD44780_PrintStr(" +01 +05 +10 +20");
              HD44780_SetCursor(0, 0);
              sprintf(printout, "%s:%d", time_print[time_state], time[time_state]);
              HD44780_PrintStr(printout);
              HD44780_SetCursor(9, 0);
              HD44780_PrintStr("->  <-");
              HD44780_SetCursor(cursor_x, cursor_y);
              HD44780_Blink();
          }
          else if(screen == CONFIRMING){
              HD44780_SetCursor(0,0);
              sprintf(printout, "%d:%d:%d", time[0], time[1], time[2]);
              HD44780_PrintStr(printout);
              HD44780_SetCursor(0,1);
              HD44780_PrintStr("  Start   Back");
              HD44780_SetCursor(cursor_x, 1);
              HD44780_Blink();
          }
          else if(screen == COUNTING){
              HD44780_SetCursor(0,0);
              sprintf(printout, "Left:   Pauses:%d", remaining_pause);
              HD44780_PrintStr(printout);
              HD44780_SetCursor(0,1);
              sprintf(printout, "%d:%d:%d", time_remaining/1000/3600,
                      time_remaining/1000%3600/60, time_remaining/1000%3600%60);
              HD44780_PrintStr(printout);
              HD44780_SetCursor(cursor_x, 0);
              HD44780_Blink();
          }
          else if(screen == PAUSING){
              HD44780_SetCursor(0,0);
              HD44780_PrintStr(" Restart");
              HD44780_SetCursor(0,1);
              sprintf(printout, "%d:%d:%d", time_remaining/1000/3600,
                      time_remaining/1000%3600/60, time_remaining/1000%3600%60);
              HD44780_PrintStr(printout);
              HD44780_SetCursor(0,0);
              HD44780_Blink();
          }
          else{
              HD44780_SetCursor(0,0);
              HD44780_PrintStr("Over");
              HD44780_SetCursor(0,1);
              HD44780_PrintStr(" Restart");
              HD44780_SetCursor(0,1);
              HD44780_Blink();
          }
          lastRefreshTick = now;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1250 - servo_state*500);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(MYPWM_GPIO_Port, MYPWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MYPWM_Pin */
  GPIO_InitStruct.Pin = MYPWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MYPWM_GPIO_Port, &GPIO_InitStruct);

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
