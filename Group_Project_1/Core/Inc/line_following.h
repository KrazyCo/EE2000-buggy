/*
 * line_following.c
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#ifndef INC_LINE_FOLLOWING_C_
#define INC_LINE_FOLLOWING_C_

#include "main.h"

#define left_slight_veer_step 5
#define left_strong_veer_step 25
#define left_veer_decay 40

#define right_slight_veer_step 5
#define right_strong_veer_step 25
#define right_veer_decay 40

#define LEFT_SENSOR_PIN   GPIO_PIN_4
#define LEFT_SENSOR_PORT  GPIOB
#define RIGHT_SENSOR_PIN  GPIO_PIN_10
#define RIGHT_SENSOR_PORT GPIOA
#define MIDDLE_SENSOR_PIN  GPIO_PIN_10
#define MIDDLE_SENSOR_PORT GPIOB

enum Direction {
	FORWARD, LEFT, RIGHT
};

extern int lap_count;

void line_following_init();
void line_following_loop(TIM_HandleTypeDef *htim);
void lap_passed();

#endif /* INC_LINE_FOLLOWING_C_ */
