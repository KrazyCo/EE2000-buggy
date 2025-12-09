/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
void move_backward(uint16_t speed);
void turn_left(uint16_t speed);
void turn_right(uint16_t speed);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Btn1_A0_Pin GPIO_PIN_0
#define Btn1_A0_GPIO_Port GPIOA
#define Btn2_A1_Pin GPIO_PIN_1
#define Btn2_A1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Btn3_A2_Pin GPIO_PIN_4
#define Btn3_A2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Btn4_A3_Pin GPIO_PIN_0
#define Btn4_A3_GPIO_Port GPIOB
#define Middle_sensor_Pin GPIO_PIN_10
#define Middle_sensor_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_8
#define Echo_GPIO_Port GPIOA
#define Trig_Pin GPIO_PIN_9
#define Trig_GPIO_Port GPIOA
#define Right_sensor_Pin GPIO_PIN_10
#define Right_sensor_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Motor_Output_Pin GPIO_PIN_3
#define Motor_Output_GPIO_Port GPIOB
#define Left_sensor_Pin GPIO_PIN_4
#define Left_sensor_GPIO_Port GPIOB
#define Motor_OutputB5_Pin GPIO_PIN_5
#define Motor_OutputB5_GPIO_Port GPIOB
#define LED_D10_PB6_Pin GPIO_PIN_6
#define LED_D10_PB6_GPIO_Port GPIOB
#define OLED_SCK_D15_Pin GPIO_PIN_8
#define OLED_SCK_D15_GPIO_Port GPIOB
#define OLED_SDA_D14_Pin GPIO_PIN_9
#define OLED_SDA_D14_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
