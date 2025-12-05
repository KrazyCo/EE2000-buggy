/*
 * line_following.c
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#include "line_following.h"
#include "motor_control.h"
#include "config.h"
#include "main.h"
#include "ssd1306.h"

enum Direction last_turn_direction;
int left_turn_amount;
int right_turn_amount;
int lap_count;

void line_following_init() {
	last_turn_direction = FORWARD;
	left_turn_amount = 0;
	right_turn_amount = 0;
	lap_count = 0;
}

void line_following_loopk(TIM_HandleTypeDef *htim1) {
	// Read initial IR sensor values
	GPIO_PinState left_val = HAL_GPIO_ReadPin(LEFT_SENSOR_PORT,
			LEFT_SENSOR_PIN);
	GPIO_PinState right_val = HAL_GPIO_ReadPin(RIGHT_SENSOR_PORT,
			RIGHT_SENSOR_PIN);
	GPIO_PinState middle_val = HAL_GPIO_ReadPin(MIDDLE_SENSOR_PORT,
				MIDDLE_SENSOR_PIN);

	if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_SET && middle_val == GPIO_PIN_SET) {
		// Black Black Black - buggy on track so move forwards
		move_forwards(htim1);
		last_turn_direction = FORWARD;
		left_turn_amount = 0;
		right_turn_amount = 0;
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_SET && middle_val == GPIO_PIN_SET) {
		// White Black Black - buggy is veering left slightly
		right_turn_amount += right_slight_veer_step;
		if (right_turn_amount > FORWARDS_RIGHT_MOTOR_SPEED) {
			right_turn_amount = FORWARDS_RIGHT_MOTOR_SPEED;
		}
		veer_right(htim1, right_turn_amount);
		last_turn_direction = RIGHT;
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_SET && middle_val == GPIO_PIN_RESET) {
		// White White Black - buggy may be veering left strongly or finished a lap
		if (last_turn_direction == RIGHT) {
			right_turn_amount += right_strong_veer_step;
			if (right_turn_amount > FORWARDS_RIGHT_MOTOR_SPEED) {
				right_turn_amount = FORWARDS_RIGHT_MOTOR_SPEED;
			}
			veer_right(htim1, right_turn_amount);
		} else if (last_turn_direction == FORWARD) {
			lap_passed();
		}
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_RESET && middle_val == GPIO_PIN_RESET) {
		// White White White - buggy should have finished a lap - do not reset turning
		lap_passed();
	} else if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_RESET && middle_val == GPIO_PIN_SET) {
		// Black Black White - buggy is veering left slightly
		left_turn_amount += left_slight_veer_step;
		if (left_turn_amount > FORWARDS_LEFT_MOTOR_SPEED) {
			left_turn_amount = FORWARDS_LEFT_MOTOR_SPEED;
		}
		veer_left(htim1, left_turn_amount);
		last_turn_direction = LEFT;
	} else if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_RESET && middle_val == GPIO_PIN_RESET) {
		// Black White White - buggy may be veering left strongly or finished a lap
		if (last_turn_direction == LEFT) {
			left_turn_amount += left_strong_veer_step;
			if (left_turn_amount > FORWARDS_LEFT_MOTOR_SPEED) {
				left_turn_amount = FORWARDS_LEFT_MOTOR_SPEED;
			}
			veer_left(htim1, left_turn_amount);
		} else if (last_turn_direction == FORWARD) {
			lap_passed();
		}
	}

}

void lap_passed() {
	lap_count += 1;
}
