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
#include "tinsel.h"
#include "button.h"

#include "effects_service.h"
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
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim3_ch4_up;

/* USER CODE BEGIN PV */
enum { garland_length = 50 };
enum { btn_double_click_pause_ms = 200 };
enum { dummy_arg = 0 };
enum { garland_update_period_ms = 50 };
enum { garland_default_brightness = 255 };

SMARTLED_DEF(garland, garland_length);

struct button btn;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void garland_start(smartled_t *garland)
{
    HAL_TIM_PWM_Start_DMA(&htim3,
                          TIM_CHANNEL_4,
                          (uint32_t *) garland->pulses_buffer,
                          2 * SMARTLED_PULSES_PER_LED);
}

static void garland_stop(smartled_t *garland)
{
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
}

enum { auto_switch_check_period_ms = 10 };
enum { auto_switch_delay_period_ms = 60000 };
enum { auto_switch_brightness_step = 1 };

typedef enum {
    auto_switch_state_initial,
    auto_switch_state_wait_for_falling,
    auto_switch_state_falling,
    auto_switch_state_rising
} auto_switch_state_t;

static const bool random_effect_choice = false;

static volatile bool auto_switch_flag = true;
static volatile uint32_t auto_switch_counter;
static volatile auto_switch_state_t auto_switch_state = auto_switch_state_initial;

static void auto_switch_routine(uint32_t data)
{
    tinsel_add_task_timer(auto_switch_routine, dummy_arg, auto_switch_check_period_ms);

    if (!auto_switch_flag)
    {
        if (smartled_get_brightness(&garland) != garland_default_brightness)
        {
            smartled_set_brightness(&garland, garland_default_brightness);
            auto_switch_state = auto_switch_state_initial;
        }

        return;
    }

    switch (auto_switch_state)
    {
        case auto_switch_state_initial:
            smartled_set_brightness(&garland, garland_default_brightness);
            auto_switch_counter = auto_switch_delay_period_ms / auto_switch_check_period_ms;
            auto_switch_state = auto_switch_state_wait_for_falling;
            break;

        case auto_switch_state_wait_for_falling:
            if (!auto_switch_counter)
                auto_switch_state = auto_switch_state_falling;
            else
                auto_switch_counter--;
            break;

        case auto_switch_state_falling:
            if (smartled_get_brightness(&garland) > auto_switch_brightness_step)
            {
                smartled_set_brightness(&garland, smartled_get_brightness(&garland) - auto_switch_brightness_step);
            }
            else
            {
                smartled_set_brightness(&garland, 0);
                effects_service_next_effect(random_effect_choice);
                auto_switch_state = auto_switch_state_rising;
            }

            break;

        case auto_switch_state_rising:
            if (smartled_get_brightness(&garland) < garland_default_brightness - auto_switch_brightness_step)
            {
                smartled_set_brightness(&garland, smartled_get_brightness(&garland) + auto_switch_brightness_step);
            }
            else
            {
                smartled_set_brightness(&garland, garland_default_brightness);
                auto_switch_counter = auto_switch_delay_period_ms / auto_switch_check_period_ms;
                auto_switch_state = auto_switch_state_wait_for_falling;
            }

            break;
    }
}

static void garland_routine(uint32_t data)
{
    effects_service_play_currect_effect(&garland);
    smartled_flush(&garland);
}

static void button_routine(uint32_t data)
{
    tinsel_add_task_timer(button_routine, dummy_arg, 1);
    
    button_check(&btn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim17) {
        tinsel_timer_check();
    }
}

static void foo(void)
{
    if (!smartled_next(&garland)) {
        tinsel_add_task_timer(garland_routine,
                              dummy_arg,
                              garland_update_period_ms);
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef * htim)
{
    if (htim == &htim3) {
        foo();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim)
{
    if (htim == &htim3) {
        foo();
    }
}

static bool btn_read(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET;
}

static void btn_click_callback(uint8_t clicks)
{
    if (clicks == 1)
    {
        auto_switch_state = auto_switch_state_initial;
        effects_service_next_effect(false);
    }
    
    if (clicks == 2)
    {
        auto_switch_state = auto_switch_state_initial;
        effects_service_next_effect(true);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  smartled_init(&garland,
                SMARTLED_RGB,
                garland_default_brightness,
                garland_start,
                garland_stop);

  button_init(&btn,
              btn_double_click_pause_ms,
              btn_read,
              btn_click_callback);
  
  effects_service_init();
  
  tinsel_init();
  tinsel_add_task(garland_routine, dummy_arg);
  tinsel_add_task(button_routine, dummy_arg);
  tinsel_add_task(auto_switch_routine, dummy_arg);
  
  HAL_TIM_Base_Start_IT(&htim17);
  
  return tinsel_run();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 4799;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
