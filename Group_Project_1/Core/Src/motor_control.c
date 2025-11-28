/*
 * motor_control.c
 *
 *  Created on: Nov 21, 2025
 *      Author: conor
 */

#include "motor_control.h"
#include "ssd1306.h"

void move_forwards(TIM_HandleTypeDef *htim1) {
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, FORWARDS_LEFT_MOTOR_SPEED); // Left
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, FORWARDS_RIGHT_MOTOR_SPEED); // Right
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Left DIR = forward
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // Right DIR = forward
}

void veer_left(TIM_HandleTypeDef *htim1, uint16_t veer_amount) {
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, FORWARDS_LEFT_MOTOR_SPEED - veer_amount); // Left motor (slow)
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, FORWARDS_RIGHT_MOTOR_SPEED); // Right motor (fast)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Left DIR = forward
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // Right DIR = forward
}

void veer_right(TIM_HandleTypeDef *htim1, uint16_t veer_amount) {
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, FORWARDS_LEFT_MOTOR_SPEED); // Left motor (fast)
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, FORWARDS_RIGHT_MOTOR_SPEED - veer_amount); // Right motor (slow)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Left DIR = forward
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // Right DIR = forward
}

void stop_motors(TIM_HandleTypeDef *htim1) {
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, 0);
}
