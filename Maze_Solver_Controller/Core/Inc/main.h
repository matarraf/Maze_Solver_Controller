/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#define IR_in_1_Pin GPIO_PIN_0
#define IR_in_1_GPIO_Port GPIOC
#define IR_in_2_Pin GPIO_PIN_1
#define IR_in_2_GPIO_Port GPIOC
#define IR_in_3_Pin GPIO_PIN_2
#define IR_in_3_GPIO_Port GPIOC
#define IR_in_4_Pin GPIO_PIN_3
#define IR_in_4_GPIO_Port GPIOC
#define IR_in_5_Pin GPIO_PIN_4
#define IR_in_5_GPIO_Port GPIOC
#define IR_in_6_Pin GPIO_PIN_5
#define IR_in_6_GPIO_Port GPIOC
#define IR_in_7_Pin GPIO_PIN_6
#define IR_in_7_GPIO_Port GPIOC
#define IR_in_8_Pin GPIO_PIN_7
#define IR_in_8_GPIO_Port GPIOC

#define Servo_PWM_Pin GPIO_PIN_0
#define Servo_PWM_GPIO_Port GPIOA

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

#define MTOR_L_SPD_Pin GPIO_PIN_6
#define MTOR_L_SPD_GPIO_Port GPIOA
#define MTOR_R_SPD_Pin GPIO_PIN_7
#define MTOR_R_SPD_GPIO_Port GPIOA

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#define MOTOR_L_DIR_Pin GPIO_PIN_4
#define MOTOR_L_DIR_GPIO_Port GPIOB
#define MTOR_R_DIR_Pin GPIO_PIN_5
#define MTOR_R_DIR_GPIO_Port GPIOB

#define US_echo_Pin GPIO_PIN_6
#define US_echo_GPIO_Port GPIOB
#define US_trig_Pin GPIO_PIN_7
#define US_trig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Optional aliases for cleaner naming in user code */
#define MOTOR_R_DIR_Pin        MTOR_R_DIR_Pin
#define MOTOR_R_DIR_GPIO_Port  MTOR_R_DIR_GPIO_Port
#define MOTOR_L_SPD_Pin        MTOR_L_SPD_Pin
#define MOTOR_L_SPD_GPIO_Port  MTOR_L_SPD_GPIO_Port
#define MOTOR_R_SPD_Pin        MTOR_R_SPD_Pin
#define MOTOR_R_SPD_GPIO_Port  MTOR_R_SPD_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
