/*
 * motor_control.h
 *
 *  Created on: Nov 21, 2025
 *      Author: conor
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "ssd1306.h"

//TIM_HandleTypeDef htim; // defined in main.c

#define FORWARDS_LEFT_MOTOR_SPEED 400
#define FORWARDS_RIGHT_MOTOR_SPEED 400

void move_forwards(TIM_HandleTypeDef *htim);
void veer_left(TIM_HandleTypeDef *htim, uint16_t veer_amount);
void veer_right(TIM_HandleTypeDef *htim, uint16_t veer_amount);
void stop_motors(TIM_HandleTypeDef *htim);

#endif /* INC_MOTOR_CONTROL_H_ */
