/*
 * motor_control.h
 *
 *  Created on: Nov 21, 2025
 *      Author: conor
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "ssd1306.h"

//TIM_HandleTypeDef htim1; // defined in main.c

#define FORWARDS_LEFT_MOTOR_SPEED 400
#define FORWARDS_RIGHT_MOTOR_SPEED 400

#define LEFT_SENSOR_PIN   GPIO_PIN_4
#define LEFT_SENSOR_PORT  GPIOB
#define RIGHT_SENSOR_PIN  GPIO_PIN_10
#define RIGHT_SENSOR_PORT GPIOA
#define MIDDLE_SENSOR_PIN  GPIO_PIN_10
#define MIDDLE_SENSOR_PORT GPIOB

void move_forwards(TIM_HandleTypeDef *htim);
void veer_left(TIM_HandleTypeDef *htim, uint16_t veer_amount);
void veer_right(TIM_HandleTypeDef *htim, uint16_t veer_amount);
void stop_motors(TIM_HandleTypeDef *htim);

#endif /* INC_MOTOR_CONTROL_H_ */
