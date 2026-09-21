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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* TIM6: 1 ms tick. All application state is updated in this interrupt. */
#define KEY_DEBOUNCE_MS  20U
#define KEY_DOUBLE_MS    300U
#define KEY_LONG_MS      800U
#define SERVO_CENTER_US  1500U

/* One complete LED cycle / servo round trip: 1 Hz, 2 Hz, 0.5 Hz. */
static const uint16_t half_period_ms[] = {500U, 250U, 1000U};
/* Pulse-width offsets from center, not calibrated mechanical angles. */
static const uint16_t amplitude_us[] = {250U, 500U, 750U};

static uint32_t app_ms;
static uint16_t phase_ms;
static uint8_t speed_index;
static uint8_t amplitude_index;
static uint8_t running = 1U;
static uint8_t servo_divider;

static uint8_t key_candidate;
static uint8_t key_stable;
static uint8_t key_debounce;
static uint8_t key_long_sent;
static uint8_t click_pending;
static uint8_t second_press;
static uint32_t press_ms;
static uint32_t release_ms;

static void Servo_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < 500U) pulse_us = 500U;
    if (pulse_us > 2500U) pulse_us = 2500U;
    /* TIM1 counts at 5 MHz: 5 counts per microsecond. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)pulse_us * 5U);
}

static void Servo_Update(void)
{
    /* Triangle trajectory: center -> right -> center -> left -> center. */
    int32_t q = (int32_t)half_period_ms[speed_index] / 2;
    int32_t p = (int32_t)phase_ms;
    int32_t a = (int32_t)amplitude_us[amplitude_index];
    int32_t offset;

    if (p < q)
        offset = a * p / q;
    else if (p < 3 * q)
        offset = a * (2 * q - p) / q;
    else
        offset = a * (p - 4 * q) / q;

    Servo_SetPulseUs((uint16_t)((int32_t)SERVO_CENTER_US + offset));
}

static void Wave_Reset(void)
{
    phase_ms = 0U;
    servo_divider = 0U;
    /* PB12 is active high on this board. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,
                     running ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Servo_SetPulseUs(SERVO_CENTER_US);
}

static void On_ShortPress(void)
{
    speed_index = (uint8_t)((speed_index + 1U) % 3U);
    Wave_Reset();
}

static void On_DoublePress(void)
{
    amplitude_index = (uint8_t)((amplitude_index + 1U) % 3U);
    if (running) Servo_Update();
}

static void On_LongPress(void)
{
    running = (uint8_t)!running;
    /* Off: LED off and servo holds center; PWM remains active. */
    Wave_Reset();
}

static void Key_Scan1ms(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)
                       == GPIO_PIN_RESET);

    /* Wait for the double-click window before reporting one short press. */
    if (click_pending && !key_stable &&
        (uint32_t)(app_ms - release_ms) >= KEY_DOUBLE_MS)
    {
        click_pending = 0U;
        On_ShortPress();
    }

    if (pressed != key_candidate)
    {
        key_candidate = pressed;
        key_debounce = 0U;
    }
    else if (key_debounce < KEY_DEBOUNCE_MS)
    {
        ++key_debounce;
    }

    if (key_debounce == KEY_DEBOUNCE_MS && key_stable != key_candidate)
    {
        key_stable = key_candidate;
        if (key_stable) /* Debounced press. */
        {
            press_ms = app_ms;
            key_long_sent = 0U;
            second_press = click_pending;
            click_pending = 0U;
        }
        else /* Debounced release. */
        {
            if (!key_long_sent)
            {
                if (second_press)
                    On_DoublePress();
                else
                {
                    click_pending = 1U;
                    release_ms = app_ms;
                }
            }
            second_press = 0U;
            key_long_sent = 0U;
        }
    }

    if (key_stable && pressed && !key_long_sent &&
        (uint32_t)(app_ms - press_ms) >= KEY_LONG_MS)
    {
        key_long_sent = 1U;
        click_pending = 0U;
        second_press = 0U;
        On_LongPress();
    }
}

static void App_Tick1ms(void)
{
    ++app_ms;
    Key_Scan1ms(); /* Keep scanning even when blinking/swinging is off. */
    if (!running) return;

    uint16_t half = half_period_ms[speed_index];
    if (++phase_ms >= (uint16_t)(2U * half)) phase_ms = 0U;

    if (phase_ms == 0U)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    else if (phase_ms == half)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    /* Refresh the servo target every 10 ms; TIM1 generates PWM itself. */
    if (++servo_divider >= 10U)
    {
        servo_divider = 0U;
        Servo_Update();
    }
}

static void App_Init(void)
{
    /* Ignore a key already held at power-up until it is released. */
    key_candidate = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
    key_stable = key_candidate;
    key_long_sent = key_candidate;
    Wave_Reset();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    /* Discard any update flag left by timer initialization. */
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
        Error_Handler();
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
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        App_Tick1ms();
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

