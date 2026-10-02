/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SG90 servo motor control using TIM3 Channel 2
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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * TIM3 설정
 *
 * TIM3 clock       = 16 MHz
 * Prescaler        = 15
 * Counter frequency= 1 MHz
 * 1 timer count    = 1 us
 * Period           = 19999
 * PWM period       = 20000 us = 20 ms = 50 Hz
 */

/* SG90 안전 시험 범위 */
#define SERVO_MIN_US             1200U
#define SERVO_CENTER_US          1500U
#define SERVO_MAX_US             1800U

/* 시험용 위치 */
#define SERVO_LEFT_US            1200U
#define SERVO_RIGHT_US           1800U

/* 래치 제어용 예비 위치 */
#define SERVO_LOCK_US            1300U
#define SERVO_RELEASE_US         1750U

/* 서보 이동 후 대기시간 */
#define SERVO_MOVE_DELAY_MS      1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/*
 * 래치가 한 번 작동한 이후 다시 작동하지 않게 만들 때 사용할 변수
 * 현재 SG90 단독 시험에서는 사용하지 않음
 */
static uint8_t servo_triggered = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

static void Servo_SetPulseUs(uint16_t pulse_us);
static void Servo_SetAngle(uint8_t angle);
static void Servo_Lock(void);
static void Servo_Release(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  SG90 PWM 펄스 폭 설정
  * @param  pulse_us: PWM High 시간, 단위는 us
  * @retval None
  *
  * 현재 안전 범위:
  * 1200 us ~ 1800 us
  */
static void Servo_SetPulseUs(uint16_t pulse_us)
{
  /*
   * 너무 작은 값이나 큰 값이 들어오는 것을 방지한다.
   * 서보가 기계적인 끝에 강하게 부딪히는 것을 방지하기 위한 제한이다.
   */
  if (pulse_us < SERVO_MIN_US)
  {
    pulse_us = SERVO_MIN_US;
  }
  else if (pulse_us > SERVO_MAX_US)
  {
    pulse_us = SERVO_MAX_US;
  }

  /*
   * TIM3 Channel 2의 Compare 값을 변경한다.
   *
   * 타이머 카운터 1칸 = 1 us이므로
   * Compare 값 1500 = High 시간 1500 us
   */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
}


/**
  * @brief  각도를 PWM 펄스 폭으로 변환
  * @param  angle: 0 ~ 180
  * @retval None
  *
  * 현재 설정:
  * angle 0   -> 1200 us
  * angle 90  -> 1500 us
  * angle 180 -> 1800 us
  *
  * 실제 SG90의 정확한 기계적 각도는 제품마다 다를 수 있다.
  */
static void Servo_SetAngle(uint8_t angle)
{
  uint32_t pulse_us;

  if (angle > 180U)
  {
    angle = 180U;
  }

  pulse_us =
      SERVO_MIN_US
      + (((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180U);

  Servo_SetPulseUs((uint16_t)pulse_us);
}


/**
  * @brief  래치 잠금 위치
  * @retval None
  */
static void Servo_Lock(void)
{
  Servo_SetPulseUs(SERVO_LOCK_US);
}


/**
  * @brief  래치 해제 위치
  * @retval None
  */
static void Servo_Release(void)
{
  Servo_SetPulseUs(SERVO_RELEASE_US);
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

  /*
   * 모든 주변장치를 초기화하고
   * Flash interface와 SysTick을 초기화한다.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 시스템 클럭 설정 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 설정된 주변장치 초기화 */
  MX_GPIO_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /*
   * PWM 출력 시작 전에 중앙 위치의 Compare 값을 설정한다.
   */
  Servo_SetPulseUs(SERVO_CENTER_US);

  /*
   * TIM3 Channel 2 PWM 출력 시작
   *
   * 이 코드가 실행되어야 PC7/D9 핀에서 PWM 신호가 출력된다.
   */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * SG90이 초기 중앙 위치로 이동할 시간을 준다.
   */
  HAL_Delay(1500);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /*
   * USER 버튼 초기화
   * 현재 서보 시험에는 사용하지 않지만 CubeMX 기본 설정을 유지한다.
   */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /*
   * COM1 초기화
   * 현재 서보 시험에는 사용하지 않지만 CubeMX 기본 설정을 유지한다.
   */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * 1. 왼쪽 위치
     * 약 1200 us
     */
    Servo_SetPulseUs(SERVO_LEFT_US);
    HAL_Delay(SERVO_MOVE_DELAY_MS);

    /*
     * 2. 중앙 위치
     * 약 1500 us
     */
    Servo_SetPulseUs(SERVO_CENTER_US);
    HAL_Delay(SERVO_MOVE_DELAY_MS);

    /*
     * 3. 오른쪽 위치
     * 약 1800 us
     */
    Servo_SetPulseUs(SERVO_RIGHT_US);
    HAL_Delay(SERVO_MOVE_DELAY_MS);

    /*
     * 4. 다시 중앙 위치
     */
    Servo_SetPulseUs(SERVO_CENTER_US);
    HAL_Delay(SERVO_MOVE_DELAY_MS);

    /*
     * 내장 LED 상태 반전
     * 메인 반복문이 동작하고 있다는 것을 확인하기 위한 표시
     */
    BSP_LED_Toggle(LED_GREEN);

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

  /**
    * 메인 내부 레귤레이터 전압 설정
    */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /**
    * HSI 내부 오실레이터 설정
    *
    * HSI = 16 MHz
    * PLL 사용 안 함
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * CPU, AHB, APB 클럭 설정
    *
    * SYSCLK = 16 MHz
    * HCLK   = 16 MHz
    * PCLK1  = 16 MHz
    * TIM3   = 16 MHz
    */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK
      | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */

  /*
   * TIM3 입력 클럭 = 16 MHz
   *
   * Prescaler = 15
   * 카운터 클럭 = 16 MHz / (15 + 1)
   *             = 1 MHz
   *
   * 한 카운트 시간 = 1 us
   *
   * Period = 19999
   * PWM 주기 = (19999 + 1) × 1 us
   *          = 20000 us
   *          = 20 ms
   *
   * PWM 주파수 = 50 Hz
   */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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

  /*
   * TIM3 Channel 2 PWM 설정
   *
   * Pulse = 1500
   * High 시간 = 1500 us
   * 서보 중앙 위치
   */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(
          &htim3,
          &sConfigOC,
          TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

  /*
   * PC7 핀을 TIM3_CH2 Alternate Function으로 설정한다.
   */
  HAL_TIM_MspPostInit(&htim3);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

/*
 * 이후 열감지센서와 부저를 연결할 때 사용할 수 있는 예시 함수
 *
 * 현재는 호출하지 않으므로 실제 동작에는 영향을 주지 않는다.
 */
static void Trigger_Latch_Once(void)
{
  /*
   * 이미 한 번 작동했다면 다시 작동하지 않는다.
   */
  if (servo_triggered != 0U)
  {
    return;
  }

  /*
   * 실제 시스템에서는 이 앞에 다음 동작이 들어간다.
   *
   * Buzzer_On();
   * HAL_Delay(1000);
   * Buzzer_Off();
   */

  Servo_Release();
  HAL_Delay(500);

  servo_triggered = 1U;
}

/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /*
   * 오류 발생 시 인터럽트를 비활성화하고
   * 내장 LED를 빠르게 점멸한다.
   */
  __disable_irq();

  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);

    /*
     * 인터럽트가 비활성화된 상태에서는 HAL_Delay가
     * 정상 동작하지 않을 수 있으므로 단순 지연 반복문을 사용한다.
     */
    for (volatile uint32_t delay = 0U;
         delay < 200000U;
         delay++)
    {
      /* 대기 */
    }
  }

  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the source file name and source line number
  * @param  file: source file name
  * @param  line: source line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  (void)file;
  (void)line;

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
