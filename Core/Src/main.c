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
#include <string.h>
#include <math.h>

#include "tinsel.h"
#include "button.h"
#include "colorspaces.h"
#include "smartled.h"
#include "perlin.h"
#include "utils.h"
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
enum { garland_effect_shared_data_size = 12 };
enum { auto_switch_period_ms = 60000 };
enum { garland_update_period_ms = 50 };
enum { garland_max_brightness = 255 };

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
typedef void (*garland_setup_handler)(void *, const void *);
typedef void (*garland_loop_handler)(struct SmartLED *, void *);

typedef struct {
    garland_setup_handler setup;
    garland_loop_handler loop;
    const void *initial;
} garland_effect_t;

typedef struct {
    uint32_t hue;
} running_rainbow_data_t;

STATIC_ASSERT(sizeof(running_rainbow_data_t) <= garland_effect_shared_data_size);

static void running_rainbow_setup(void *raw_data, const void *initial)
{
    running_rainbow_data_t * const data = (running_rainbow_data_t *) raw_data;

    data->hue = 0;
}

static void running_rainbow_loop(struct SmartLED *leds, void *raw_data)
{
    uint32_t i;
    running_rainbow_data_t * const data = (running_rainbow_data_t *) raw_data;
    
    for (i = 0; i < leds->length; i++)
    {
        SmartLED_Set_HSV(leds, i, hsv((data->hue+i) % full_hue, max_whiteness, max_value));
    }
        
    data->hue = (data->hue + 6) % full_hue;
}

typedef struct {
    uint32_t hue;
} all_rainbow_data_t;

STATIC_ASSERT(sizeof(all_rainbow_data_t) <= garland_effect_shared_data_size);

static void all_rainbow_setup(void *raw_data, const void *initial)
{
    all_rainbow_data_t * const data = (all_rainbow_data_t *) raw_data;

    data->hue = 0;
}

static void all_rainbow_loop(struct SmartLED *leds, void *raw_data)
{
    uint32_t i;
    all_rainbow_data_t * const data = (all_rainbow_data_t *) raw_data;
    
    for (i = 0; i < leds->length; i++)
        SmartLED_Set_HSV(leds, i, hsv(data->hue, max_whiteness, max_value));
        
    data->hue = (data->hue + 1) % full_hue;
}

typedef struct {
    RGB_t col[2];
    uint32_t offset;
} perlin_data_t;

STATIC_ASSERT(sizeof(perlin_data_t) <= garland_effect_shared_data_size);

static void perlin_setup(void *raw_data, const void *initial)
{
    memcpy(raw_data, initial, sizeof(perlin_data_t));
}

static float sigmoid(float x)
{
    const float threshold = 0.15f;
    
    if (fabs(x - 0.5f) < threshold)
    {
        return 0.5f * (1.0f + (x - 0.5f) / threshold);
    }
    
    return (x < 0.5f) ? 0.0f : 1.0f;
}

static void perlin_loop(struct SmartLED *leds, void *raw_data)
{
    perlin_data_t * const data = (perlin_data_t *) raw_data;
    uint32_t i;
    const uint32_t vertical_size = 1048576; /* must be a power of two */
    
    for (i = 0; i < leds->length; i++)
    {
        float coef = perlin(i, data->offset, leds->length, vertical_size, 10);
        coef = sigmoid((coef + 1.0f) / 2.0f);

        RGB_t color = (RGB_t)
        {
            (uint8_t) lerp((float) data->col[0].r, (float) data->col[1].r, coef),
            (uint8_t) lerp((float) data->col[0].g, (float) data->col[1].g, coef),
            (uint8_t) lerp((float) data->col[0].b, (float) data->col[1].b, coef)
        };
        
        SmartLED_Set_RGB(leds, i, color);
    }
    
    data->offset = (data->offset + 1) & (vertical_size - 1);
}

static const perlin_data_t perlin_initials[] =
{
    { {(RGB_t) {255, 0,   255}, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {255, 255, 0  }, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {0  , 255, 0  }, (RGB_t) {255, 0,   255}}, 0 },
    { {(RGB_t) {180, 0,   0  }, (RGB_t) {255, 255, 100}}, 0 }, /* flame */
};

static const garland_effect_t garland_effects[] = {
    { running_rainbow_setup, running_rainbow_loop, NULL },
    { all_rainbow_setup, all_rainbow_loop, NULL },
    
    { perlin_setup, perlin_loop, &(perlin_initials[0]) },
    { perlin_setup, perlin_loop, &(perlin_initials[1]) },
    { perlin_setup, perlin_loop, &(perlin_initials[2]) },
    { perlin_setup, perlin_loop, &(perlin_initials[3]) },
};

static volatile uint32_t current_effect_idx = 0;
static uint8_t garland_effect_shared_data[garland_effect_shared_data_size];

static volatile bool auto_switch = true;

static void garland_select_effect(uint32_t idx)
{
    current_effect_idx = idx;
    (garland_effects[idx].setup)(garland_effect_shared_data, garland_effects[idx].initial);
}

static void garland_next_effect(void)
{
    uint32_t idx = current_effect_idx + 1;

    if (idx >= sizeof(garland_effects) / sizeof(garland_effects[0]))
    {
        idx = 0;
    }

    garland_select_effect(idx);
}

static void garland_play_currect_effect(void)
{
    (garland_effects[current_effect_idx].loop)(&garland, garland_effect_shared_data);
}
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

static void auto_switch_routine(uint32_t data)
{
    tinsel_add_task_timer(auto_switch_routine,
                          dummy_arg,
                          auto_switch_period_ms);

    if (auto_switch)
    {
        garland_next_effect();
    }
}

static void garland_routine(uint32_t data)
{
    garland_play_currect_effect();
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

static void foo(void)
{
    if (!SmartLED_Next(&garland)) {
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
        garland_next_effect();
    }
    
    if (clicks == 2)
    {
        auto_switch = !auto_switch;
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
                garland_max_brightness,
                leds_buffer,
                pulses_buffer,
                garland_start,
                garland_stop);
  
  button_init(&btn,
              btn_double_click_pause_ms,
              btn_read,
              btn_click_callback);
  
  garland_select_effect(0);
  
  tinsel_init();
  tinsel_add_task(garland_routine, dummy_arg);
  tinsel_add_task(button_routine, dummy_arg);
  tinsel_add_task_timer(auto_switch_routine, dummy_arg, auto_switch_period_ms);
  
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
