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
#include <stdlib.h>
#include <stdbool.h>

#include "tinsel.h"
#include "button.h"
#include "colorspaces.h"
#include "smartled.h"
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

enum {
    full_hue = 360,
    max_whiteness = 100,
    max_value = 100
};

static struct SmartLED garland;

static uint8_t leds_buffer[garland_length * 3];
static uint8_t pulses_buffer[smartled_pulses_per_led * 2];

struct button btn;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
typedef void (*garland_effect)(struct SmartLED *, uint32_t *);

static void running_rainbow(struct SmartLED *leds, uint32_t *userdata)
{
    uint32_t i;
    uint32_t hue = *userdata;
    
    for (i = 0; i < leds->length; i++)
        SmartLED_Set_HSV(leds, i, hsv((hue+i) % full_hue, max_whiteness, max_value));
        
    hue = (hue + 6) % full_hue;
    *userdata = hue;
}

static void all_rainbow(struct SmartLED *leds, uint32_t *userdata)
{
    uint32_t i;
    uint32_t hue = *userdata;
    
    for (i = 0; i < leds->length; i++)
        SmartLED_Set_HSV(leds, i, hsv(hue, max_whiteness, max_value));
        
    hue = (hue + 1) % full_hue;
    *userdata = hue;
}

static int32_t weight_average(int32_t a, int32_t b, int32_t coef, int32_t max_coef)
{
    return (a*coef + b*(max_coef - coef)) / max_coef;
}

static void complementary1(struct SmartLED *leds, uint32_t *userdata)
{
    uint32_t i;
    int16_t *params = (int16_t *) userdata;
    
    const RGB_t col[2] = {(RGB_t) {255, 128, 0},
                          (RGB_t) {0, 128, 255}};
    const uint32_t period = 32;
    
    if (params[0] >= period && params[1] == 0)
        params[1] = 1;
    
    if (params[0] <= 0 && params[1] == 1)
        params[1] = 0;
    
    params[0] += params[1] ? -1 : 1;
    
    RGB_t color_even = (RGB_t)
    {
        weight_average(col[0].r, col[1].r, params[0], period),
        weight_average(col[0].g, col[1].g, params[0], period),
        weight_average(col[0].b, col[1].b, params[0], period)
    };
    
    RGB_t color_odd = (RGB_t)
    {
        weight_average(col[1].r, col[0].r, params[0], period),
        weight_average(col[1].g, col[0].g, params[0], period),
        weight_average(col[1].b, col[0].b, params[0], period)
    };
    
    for (i = 0; i < leds->length; i++)
    {
        SmartLED_Set_RGB(leds, i, (i % 2) ? color_odd : color_even);
    }
    
    *userdata = *((uint32_t *) params);
}

static void complementary2(struct SmartLED *leds, uint32_t *userdata)
{
    uint32_t i;
    int16_t *params = (int16_t *) userdata;
    
    const RGB_t col[2] = {(RGB_t) {128, 255, 0},
                          (RGB_t) {128, 0, 255}};
    const uint32_t period = 32;
    
    if (params[0] >= period && params[1] == 0)
        params[1] = 1;
    
    if (params[0] <= 0 && params[1] == 1)
        params[1] = 0;
    
    params[0] += params[1] ? -1 : 1;
    
    RGB_t color_even = (RGB_t)
    {
        weight_average(col[0].r, col[1].r, params[0], period),
        weight_average(col[0].g, col[1].g, params[0], period),
        weight_average(col[0].b, col[1].b, params[0], period)
    };
    
    RGB_t color_odd = (RGB_t)
    {
        weight_average(col[1].r, col[0].r, params[0], period),
        weight_average(col[1].g, col[0].g, params[0], period),
        weight_average(col[1].b, col[0].b, params[0], period)
    };
    
    for (i = 0; i < leds->length; i++)
    {
        SmartLED_Set_RGB(leds, i, (i % 2) ? color_odd : color_even);
    }
    
    *userdata = *((uint32_t *) params);
}

static garland_effect effects_list[] = {
    running_rainbow,
    all_rainbow,
    complementary1,
    complementary2,
};

static volatile uint32_t current_effect_data = 0;
static volatile uint32_t current_effect_idx = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void garland_start(struct SmartLED *garland)
{
    HAL_TIM_PWM_Start_DMA(&htim3,
                          TIM_CHANNEL_4,
                          (uint32_t *) garland->pulses_buffer,
                          2 * smartled_pulses_per_led);
}

static void garland_stop(struct SmartLED *garland)
{
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
}

static bool btn_read(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET;
}

static void btn_click_callback(uint8_t clicks)
{
    if (clicks == 1) {
        current_effect_idx++;
        current_effect_data = 0;
        
        if (current_effect_idx == sizeof(effects_list)/sizeof(effects_list[0])) {
            current_effect_idx = 0;
        }
    }
}

static void garland_routine(uint32_t data)
{
    effects_list[current_effect_idx](&garland, &current_effect_data);
    SmartLED_Flush(&garland);
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

void foo(void)
{
    if (1 == SmartLED_Next(&garland)) {
        tinsel_add_task_timer(garland_routine, dummy_arg, 50);
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
  SmartLED_Init(&garland,
                garland_length,
                leds_buffer,
                pulses_buffer,
                garland_start,
                garland_stop);
  
  button_init(&btn,
              btn_double_click_pause_ms,
              btn_read,
              btn_click_callback);
  
  tinsel_init();
  tinsel_add_task(garland_routine, dummy_arg);
  tinsel_add_task(button_routine, dummy_arg);
  
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
