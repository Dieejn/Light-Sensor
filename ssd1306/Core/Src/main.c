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
#include "ssd1306.h"
#include "fonts.h"
#include "button_detect.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_START = 0,
  STATE_MEASURE,
  STATE_THRESHOLD,
  STATE_1_MEASURE,
  STATE_1_THRESHOLD,
  STATE_2_MEASURE,
  STATE_2_THRESHOLD
} state_machine_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANGE_RESISTOR_1000  1
#define CHANGE_RESISTOR_100   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static uint8_t rows[3] = {0, 20, 40};
static uint8_t menu = 0;

// state
static volatile state_button_t state_button = state_null;
static state_machine_t state_machine = STATE_START;

// string
static uint8_t string[10];

// value lux
static volatile uint16_t value = 1000;
static int8_t value_threshold[4] = {0, 0, 0, 0};
static uint16_t value_threshold_setting;

// digit position
static int8_t digit_position = 0;
static uint8_t digit_position_display[4] = {75, 82, 89, 96};

// flag
static uint8_t state_no_null;
static uint8_t change_state_null = 1;
static uint8_t flag_screen = 1;
static uint8_t flag_enable_threshold = 0;

// votage_adc
float votage = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void menu_pointer(uint8_t pointer)
{
  if (pointer == 1)
  {
    SSD1306_GotoXY(0, rows[1]);
    SSD1306_Puts(">", &Font_7x10, 1);
    SSD1306_GotoXY(0, rows[2]);
    SSD1306_Puts(" ", &Font_7x10, 1);
    SSD1306_UpdateScreen();
  }
  else
  {
    SSD1306_GotoXY(0, rows[1]);
    SSD1306_Puts(" ", &Font_7x10, 1);
    SSD1306_GotoXY(0, rows[2]);
    SSD1306_Puts(">", &Font_7x10, 1);
    SSD1306_UpdateScreen();
  }
}

void DISPLAY_Init(void)
{
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("STARTING...", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay(1000);
}

void DISPLAY_Start(void)
{
  SSD1306_GotoXY(20, 0);
  SSD1306_Puts("SECLECT MODE", &Font_7x10, 1);
  SSD1306_GotoXY(0, 20);
  SSD1306_Puts(">", &Font_7x10, 1);
  SSD1306_GotoXY(10, 20);
  SSD1306_Puts("1.MEASURE", &Font_7x10, 1);
  SSD1306_GotoXY(10, 40);
  SSD1306_Puts("2.THRESHOLD", &Font_7x10, 1);
  SSD1306_UpdateScreen();
}

void DISPLAY_SELECT_Range(void)
{
  SSD1306_GotoXY(20, 0);
  SSD1306_Puts("SELECT RANGE", &Font_7x10, 1);
  SSD1306_GotoXY(0, 20);
  SSD1306_Puts(">", &Font_7x10, 1);
  SSD1306_GotoXY(10, 20);
  SSD1306_Puts("1.RANGE1:10-100", &Font_7x10, 1);
  SSD1306_GotoXY(10, 40);
  SSD1306_Puts("2.RANGE2:10-1000", &Font_7x10, 1);
  SSD1306_UpdateScreen();
}

void STATE_BUTTON_Handle(void)
{
  if (state_button == state_null)
  {
    if (state_no_null == 1)
    {
      change_state_null = 1;
      state_no_null = 0;
    }
  }
  else
  {
    state_no_null = 1;
  }
}

void TASK_Handle(void)
{
  switch (state_machine)
  {
  case STATE_START:
    if (flag_screen == 1)
    {
    	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
      SSD1306_Clear();
      DISPLAY_Start();
      flag_screen = 0;
      menu = 1;
    }
    if ((state_button == state_mode) && (change_state_null == 1))
    {
      menu = menu + 1;
      if (menu == 3)	 menu = 1;
      menu_pointer(menu);
      change_state_null = 0;
    }

    if ((state_button == state_set) && (change_state_null == 1))
    {
      if (menu == 1)
      {
        state_machine = STATE_MEASURE;
      }
      else
      {
        state_machine = STATE_THRESHOLD;
      }
      flag_screen = 1;
      change_state_null = 0;
    }
    
    break;

  case STATE_MEASURE:
    if (flag_screen == 1)
    {
    	HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
      SSD1306_Clear();
      DISPLAY_SELECT_Range();
      flag_screen = 0;
      menu = 1;
    }

    if ((state_button == state_mode) && (change_state_null == 1))
    {
      menu = menu + 1;
      if (menu == 3)	 menu = 1;
      menu_pointer(menu);
      change_state_null = 0;
    }

    if ((state_button == state_set) && (change_state_null == 1))
    {
      if (menu == 1)
      {
        HAL_GPIO_WritePin(CHANGE_RESISTOR_GPIO_Port, CHANGE_RESISTOR_Pin, CHANGE_RESISTOR_100);
        state_machine = STATE_1_MEASURE;
      }
      else
      {
        HAL_GPIO_WritePin(CHANGE_RESISTOR_GPIO_Port, CHANGE_RESISTOR_Pin, CHANGE_RESISTOR_1000);
        state_machine = STATE_2_MEASURE;
      }
      
      flag_screen = 1;
      change_state_null = 0;
    }
    break;

  case STATE_THRESHOLD:
    if (flag_screen == 1)
    {
    	HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
      SSD1306_Clear();
      DISPLAY_SELECT_Range();
      flag_screen = 0;
      menu = 1;
    }

    if ((state_button == state_mode) && (change_state_null == 1))
    {
      menu = menu + 1;
      if (menu == 3)	 menu = 1;
      menu_pointer(menu);
      change_state_null = 0;
    }

    if ((state_button == state_set) && (change_state_null == 1))
    {
      if (menu == 1)
      {
        HAL_GPIO_WritePin(CHANGE_RESISTOR_GPIO_Port, CHANGE_RESISTOR_Pin, CHANGE_RESISTOR_100);
        state_machine = STATE_1_THRESHOLD;
      }
      else
      {
        HAL_GPIO_WritePin(CHANGE_RESISTOR_GPIO_Port, CHANGE_RESISTOR_Pin, CHANGE_RESISTOR_1000);
        state_machine = STATE_2_THRESHOLD;
      }
      
      flag_screen = 1;
      change_state_null = 0;
    }
    break;
  case STATE_1_MEASURE:
    if (flag_screen == 1)
    {
    	HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
      SSD1306_Clear();
      SSD1306_GotoXY(20, 0);
      SSD1306_Puts("EV:10-100", &Font_7x10, 1);
      flag_screen = 0;
    }

    sprintf((char *)string, "%d Lux    ", value);
    SSD1306_GotoXY(40, 20);
    SSD1306_Puts((char *)string, &Font_7x10, 1);
    if ((state_button == state_mode) && (change_state_null == 1))
    {
      flag_screen = 1;
      state_machine = STATE_START;
      change_state_null = 0;
    }
    SSD1306_UpdateScreen();
    
    break;

  case STATE_2_MEASURE:
    if (flag_screen == 1)
    {
    	HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
      SSD1306_Clear();
      SSD1306_GotoXY(20, 0);
      SSD1306_Puts("EV:10-1000", &Font_7x10, 1);
      flag_screen = 0;
    }

    sprintf((char *)string, "%d Lux   ", value);
    SSD1306_GotoXY(40, 20);
    SSD1306_Puts((char *)string, &Font_7x10, 1);
    if ((state_button == state_mode) && (change_state_null == 1))
    {
      flag_screen = 1;
      state_machine = STATE_START;
      change_state_null = 0;
    }
    SSD1306_UpdateScreen();
    break;

  case STATE_1_THRESHOLD:
    if (flag_screen == 1)
    {
      digit_position = 0;
      SSD1306_Clear();
      SSD1306_GotoXY(20, 0);
      SSD1306_Puts("EV:10-100", &Font_7x10, 1);
      SSD1306_GotoXY(0, 20);
      SSD1306_Puts("SET VALUE:", &Font_7x10, 1);
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      flag_screen = 0;
      flag_enable_threshold = 0;
    }

    if ((state_button == state_left) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      digit_position = digit_position - 1;
      if (digit_position == -1)
      {
        digit_position = 2;
      }
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      change_state_null = 0;
    }

    if ((state_button == state_right) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      digit_position = digit_position + 1;
      if (digit_position == 3)
      {
        digit_position = 0;
      }
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      change_state_null = 0;
    }
    
    if ((state_button == state_up) && (change_state_null == 1))
    {
      value_threshold[digit_position] = value_threshold[digit_position] + 1;
      if (value_threshold[digit_position] == 10)
      {
        value_threshold[digit_position] = 0;
      }
      
      change_state_null = 0;
    }

    if ((state_button == state_down) && (change_state_null == 1))
    {
      value_threshold[digit_position] = value_threshold[digit_position] - 1;
      if (value_threshold[digit_position] == -1)
      {
        value_threshold[digit_position] = 9;
      }
      change_state_null = 0;
    }
    sprintf((char *)string, "%d%d%d", value_threshold[0], value_threshold[1], value_threshold[2]);
    SSD1306_GotoXY(75, 20);
    SSD1306_Puts((char *)string, &Font_7x10, 1);
    
    if ((state_button == state_mode) && (change_state_null == 1))
    {
      flag_screen = 1;
      state_machine = STATE_START;
      change_state_null = 0;
    }

    if ((state_button == state_set) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      value_threshold_setting = value_threshold[0] * 100 +  value_threshold[1] * 10 + value_threshold[2];
      flag_enable_threshold = 1;
      change_state_null = 0;
    }

    if(flag_enable_threshold == 1)
    {
      if(value_threshold_setting <= value)
      {
        HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 0);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
      }
      else
      {
        HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
      }
    }
    SSD1306_UpdateScreen();
    break;

  case STATE_2_THRESHOLD:
    if (flag_screen == 1)
    {
      digit_position = 0;
      SSD1306_Clear();
      SSD1306_GotoXY(20, 0);
      SSD1306_Puts("EV:10-1000", &Font_7x10, 1);
      SSD1306_GotoXY(0, 20);
      SSD1306_Puts("SET VALUE:", &Font_7x10, 1);
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      flag_screen = 0;
      flag_enable_threshold = 0;
      value_threshold_setting = 0;
    }

    if ((state_button == state_left) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      digit_position = digit_position - 1;
      if (digit_position == -1)
      {
        digit_position = 3;
      }
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      change_state_null = 0;
    }

    if ((state_button == state_right) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      digit_position = digit_position + 1;
      if (digit_position == 4)
      {
        digit_position = 0;
      }
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts("^", &Font_7x10, 1);
      change_state_null = 0;
    }
    
    if ((state_button == state_up) && (change_state_null == 1))
    {
      value_threshold[digit_position] = value_threshold[digit_position] + 1;
      if (value_threshold[digit_position] == 10)
      {
        value_threshold[digit_position] = 0;
      }
      
      change_state_null = 0;
    }

    if ((state_button == state_down) && (change_state_null == 1))
    {
      value_threshold[digit_position] = value_threshold[digit_position] - 1;
      if (value_threshold[digit_position] == -1)
      {
        value_threshold[digit_position] = 9;
      }
      change_state_null = 0;
    }

    sprintf((char *)string, "%d%d%d%d", value_threshold[0], value_threshold[1], value_threshold[2], value_threshold[3]);
    SSD1306_GotoXY(75, 20);
    SSD1306_Puts((char *)string, &Font_7x10, 1);
    

    if ((state_button == state_mode) && (change_state_null == 1))
    {
      flag_screen = 1;
      state_machine = STATE_START;
      change_state_null = 0;
    }

    if ((state_button == state_set) && (change_state_null == 1))
    {
      SSD1306_GotoXY(digit_position_display[digit_position], 30);
      SSD1306_Puts(" ", &Font_7x10, 1);
      value_threshold_setting = value_threshold[0] * 1000 +  value_threshold[1] * 100 + value_threshold[2] * 10 + value_threshold[3];
      flag_enable_threshold = 1;
      change_state_null = 0;
    }
    
    if(flag_enable_threshold == 1)
    {
      if(value_threshold_setting <= value)
      {
        HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 0);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
      }
      else
      {
        HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
      }
    }
    SSD1306_UpdateScreen();
    break;
  default:
    break;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  DISPLAY_Init();
  /* USER CODE END 2 */
uint32_t timer = 0;
HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, 1);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     state_button = button_detect();
     STATE_BUTTON_Handle();
     TASK_Handle();

     if(HAL_GetTick() - timer > 500)
     {
    	 HAL_ADC_Start(&hadc1);
    	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		if((state_machine == STATE_1_MEASURE) || (state_machine == STATE_1_THRESHOLD))
		{
			votage = (float)(HAL_ADC_GetValue(&hadc1)) * 2.2 / 4096;
			HAL_ADC_Stop(&hadc1);

			value = (uint16_t)((22.3 - (votage * 10)) / 0.0498);
			if(value > 100) value = 100;
			if(value < 10)  value = 10;
		}
		if((state_machine == STATE_2_MEASURE) || (state_machine == STATE_2_THRESHOLD))
		{
			votage = (float)(HAL_ADC_GetValue(&hadc1)) * 0.9 / 4096;
			HAL_ADC_Stop(&hadc1);
			value = (uint16_t)((4.7 - (votage*10)) / 0.00102);

			// round
			uint8_t DV = value % 10;
			if (DV >= 5) value = value + (10 - DV);
			else         value = value - DV;
			if(value > 1000) value = 1000;
			if(value < 10)  value = 10;
		}
		timer = HAL_GetTick();
     }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CHANGE_RESISTOR_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_INFOR_GPIO_Port, LED_INFOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW_LEFT_Pin SW_MODE_Pin SW_DOWN_Pin SW_UP_Pin
                           SW_RIGHT_Pin SW_SET_Pin */
  GPIO_InitStruct.Pin = SW_LEFT_Pin|SW_MODE_Pin|SW_DOWN_Pin|SW_UP_Pin
                          |SW_RIGHT_Pin|SW_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHANGE_RESISTOR_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = CHANGE_RESISTOR_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_INFOR_Pin */
  GPIO_InitStruct.Pin = LED_INFOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_INFOR_GPIO_Port, &GPIO_InitStruct);

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
