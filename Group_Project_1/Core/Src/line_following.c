/*
 * line_following.c
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#include "line_following.h"
#include "config.h"
#include "main.h"

void line_following_init() {
	last_turn_direction = FORWARD;
}

int line_following_loop() {
	// Read initial IR sensor values
	GPIO_PinState left_val = HAL_GPIO_ReadPin(LEFT_SENSOR_PORT,
			LEFT_SENSOR_PIN);
	GPIO_PinState right_val = HAL_GPIO_ReadPin(RIGHT_SENSOR_PORT,
			RIGHT_SENSOR_PIN);
	GPIO_PinState middle_val = HAL_GPIO_ReadPin(MIDDLE_SENSOR_PORT,
				MIDDLE_SENSOR_PIN);

	if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_RESET && middle_val == GPIO_PIN_RESET) {
		// Black Black Black - buggy on track so move forwards
		move_forward(STRAIGHT_SPEED);
		last_turn_direction = FORWARD;
	} else if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_RESET && middle_val == GPIO_PIN_RESET) {
		// White Black Black - buggy is veering right slightly

	}

}
