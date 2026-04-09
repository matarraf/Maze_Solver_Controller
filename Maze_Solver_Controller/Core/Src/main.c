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
#include <stdint.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* Private user code ---------------------------------------------------------*/

// ================= Servo =================
#define SERVO_TIM   (&htim2)
#define SERVO_CH    TIM_CHANNEL_1

static void servo_set_pulse_us(uint16_t pulse_us)
{
  if (pulse_us < 1000U) pulse_us = 1000U;
  if (pulse_us > 2000U) pulse_us = 2000U;
  __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CH, pulse_us);
}

static void servo_set_angle(float angle)
{
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  float pulse_us_f = 1000.0f + (angle / 180.0f) * 1000.0f;
  servo_set_pulse_us((uint16_t)(pulse_us_f + 0.5f));
}

// ================= Motor =================
#define MOTOR_TIM        (&htim3)
#define MOTOR1_PWM_CH    TIM_CHANNEL_1
#define MOTOR2_PWM_CH    TIM_CHANNEL_2

#define M1_PIN           GPIO_PIN_7
#define M1_PORT          GPIOB
#define M2_PIN           GPIO_PIN_4
#define M2_PORT          GPIOB

static void motor_set_raw(uint32_t channel, uint8_t value)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIM);
  uint32_t pulse = ((uint32_t)value * arr) / 255U;
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, channel, pulse);
}

static void motor_set_both(uint8_t value)
{
  motor_set_raw(MOTOR1_PWM_CH, value);
  motor_set_raw(MOTOR2_PWM_CH, value);
}

static void motor_set_direction(bool forward)
{
  HAL_GPIO_WritePin(M1_PORT, M1_PIN, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M2_PORT, M2_PIN, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ================= IR Sensors =================
#define IR_BLACK_IS_LOW   1U
#define IR_SENSOR_COUNT   8U

static GPIO_TypeDef* const ir_ports[IR_SENSOR_COUNT] =
{
  IR_in_1_GPIO_Port,
  IR_in_2_GPIO_Port,
  IR_in_3_GPIO_Port,
  IR_in_4_GPIO_Port,
  IR_in_5_GPIO_Port,
  IR_in_6_GPIO_Port,
  IR_in_7_GPIO_Port,
  IR_in_8_GPIO_Port
};

static const uint16_t ir_pins[IR_SENSOR_COUNT] =
{
  IR_in_1_Pin,
  IR_in_2_Pin,
  IR_in_3_Pin,
  IR_in_4_Pin,
  IR_in_5_Pin,
  IR_in_6_Pin,
  IR_in_7_Pin,
  IR_in_8_Pin
};

// idx: 0..7
static bool ir_read_black(uint8_t idx)
{
  if (idx >= IR_SENSOR_COUNT)
  {
    return false;
  }

  GPIO_PinState s = HAL_GPIO_ReadPin(ir_ports[idx], ir_pins[idx]);

#if IR_BLACK_IS_LOW
  return (s == GPIO_PIN_RESET);
#else
  return (s == GPIO_PIN_SET);
#endif
}

// bit0 = IR1 ... bit7 = IR8
static uint8_t ir_read_mask(void)
{
  uint8_t mask = 0U;

  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++)
  {
    if (ir_read_black(i))
    {
      mask |= (1U << i);
    }
  }

  return mask;
}

// ================= DWT timing =================
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t micros(void)
{
  return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

// ================= Ultrasonic =================
static void us_trigger(void)
{
  HAL_GPIO_WritePin(US_trig_GPIO_Port, US_trig_Pin, GPIO_PIN_RESET);

  for (volatile int i = 0; i < 100; i++) { __NOP(); }

  HAL_GPIO_WritePin(US_trig_GPIO_Port, US_trig_Pin, GPIO_PIN_SET);
  uint32_t t0 = micros();
  while ((micros() - t0) < 10U) { }
  HAL_GPIO_WritePin(US_trig_GPIO_Port, US_trig_Pin, GPIO_PIN_RESET);
}

static uint32_t us_echo_pulse_us(uint32_t timeout_us)
{
  uint32_t start_wait = micros();

  while (HAL_GPIO_ReadPin(US_echo_GPIO_Port, US_echo_Pin) == GPIO_PIN_RESET)
  {
    if ((micros() - start_wait) > timeout_us) return 0U;
  }

  uint32_t t_rise = micros();

  while (HAL_GPIO_ReadPin(US_echo_GPIO_Port, US_echo_Pin) == GPIO_PIN_SET)
  {
    if ((micros() - t_rise) > timeout_us) return 0U;
  }

  uint32_t t_fall = micros();
  return (t_fall - t_rise);
}

static uint32_t us_read_distance_mm(void)
{
  us_trigger();

  uint32_t pw = us_echo_pulse_us(30000U);
  if (pw == 0U) return 0U;

  return (pw * 343U) / 2000U;
}

// ================= Globals =================
volatile uint8_t  g_ir_mask = 0U;
volatile uint32_t g_us_mm   = 0U;

// ================= Bang-bang steering =================
#define MOTOR_SPEED_50_PERCENT      128U

#define SERVO_STRAIGHT_ANGLE        90.0f
#define SERVO_HARD_LEFT_ANGLE       50.0f
#define SERVO_LEFT_ANGLE            65.0f
#define SERVO_SOFT_LEFT_ANGLE       78.0f
#define SERVO_SOFT_RIGHT_ANGLE      102.0f
#define SERVO_RIGHT_ANGLE           115.0f
#define SERVO_HARD_RIGHT_ANGLE      130.0f

// Using IR2..IR7 as the 6 steering sensors
#define STEER_SENSOR_MASK  ((1U << 1) | (1U << 2) | (1U << 3) | (1U << 4) | (1U << 5) | (1U << 6))

static float g_last_steer_angle = SERVO_STRAIGHT_ANGLE;

static void steering_bang_bang_6(uint8_t ir_mask)
{
  uint8_t steer_mask = ir_mask & STEER_SENSOR_MASK;

  // No line seen -> keep previous steering command
  if (steer_mask == 0U)
  {
    servo_set_angle(g_last_steer_angle);
    return;
  }

  // Average active sensor position across IR2..IR7
  // IR2->0, IR3->1, IR4->2, IR5->3, IR6->4, IR7->5
  uint32_t sum = 0U;
  uint32_t count = 0U;

  for (uint8_t sensor = 1U; sensor <= 6U; sensor++)
  {
    if (steer_mask & (1U << sensor))
    {
      sum += (sensor - 1U);
      count++;
    }
  }

  if (count == 0U)
  {
    servo_set_angle(g_last_steer_angle);
    return;
  }

  uint32_t avg = sum / count;
  float target_angle;

  switch (avg)
  {
    case 0:  target_angle = SERVO_HARD_LEFT_ANGLE;  break;   // IR2
    case 1:  target_angle = SERVO_LEFT_ANGLE;       break;   // IR3
    case 2:  target_angle = SERVO_SOFT_LEFT_ANGLE;  break;   // IR4
    case 3:  target_angle = SERVO_SOFT_RIGHT_ANGLE; break;   // IR5
    case 4:  target_angle = SERVO_RIGHT_ANGLE;      break;   // IR6
    case 5:  target_angle = SERVO_HARD_RIGHT_ANGLE; break;   // IR7
    default: target_angle = SERVO_STRAIGHT_ANGLE;   break;
  }

  g_last_steer_angle = target_angle;
  servo_set_angle(target_angle);
}

// ================= Main control loop =================
static void control_loop(void)
{
  g_ir_mask = ir_read_mask();
  g_us_mm   = us_read_distance_mm();

  motor_set_direction(true);
  motor_set_both(MOTOR_SPEED_50_PERCENT);

  steering_bang_bang_6(g_ir_mask);

  HAL_Delay(10);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  dwt_init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  servo_set_angle(SERVO_STRAIGHT_ANGLE);
  motor_set_direction(true);
  motor_set_both(0U);

  while (1)
  {
    control_loop();
  }
}

/*------------------------------------------------------------------------------------------------------------------*/

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

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

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Initial output levels
  HAL_GPIO_WritePin(GPIOB, US_trig_Pin | M1_PIN | M2_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_in_1_Pin ... IR_in_8_Pin */
  GPIO_InitStruct.Pin = IR_in_1_Pin | IR_in_2_Pin | IR_in_3_Pin | IR_in_4_Pin
                      | IR_in_5_Pin | IR_in_6_Pin | IR_in_7_Pin | IR_in_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : US_trig_Pin M1_PIN M2_PIN */
  GPIO_InitStruct.Pin = US_trig_Pin | M1_PIN | M2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
