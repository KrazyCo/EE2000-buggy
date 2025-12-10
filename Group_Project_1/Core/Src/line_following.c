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
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"

void save_lap_time(uint32_t lap_time, uint32_t stopped_time);
uint16_t calculate_lap_speed(uint32_t lap_time, uint32_t stopped_time);

#define METERS_PER_SECOND 0.0105

enum Direction last_turn_direction;
int left_turn_amount;
int right_turn_amount;
int lap_count;
bool lap_finished = true;
bool currently_moving = false;
uint32_t lap_start_tick;
volatile uint32_t lap_ticks_stopped;
bool currently_in_lap = false;
uint32_t last_three_lap_times[3] = {0};
uint16_t last_three_lap_speeds[3] = {0};

void line_following_init() {
	last_turn_direction = FORWARD;
	left_turn_amount = 0;
	right_turn_amount = 0;
	lap_count = 0;
}

void line_following_loop(TIM_HandleTypeDef *htim) {
	if (!moving_forward) {
		if (currently_moving) {
			stop_motors(htim);
			currently_moving = false;
		}
		return;
	}
	if (object_in_path) {
		if (currently_moving) {
			stop_motors(htim);
			currently_moving = false;
		}
		return;
	}
	if (!currently_moving) {
		move_forwards(htim);
		currently_moving = true;
	}
	if (!currently_in_lap) {
		lap_start_tick = HAL_GetTick();
		lap_ticks_stopped = 0;
		currently_in_lap = true;
	}
	// Read initial IR sensor values
	taskENTER_CRITICAL();// disable interrupts
	GPIO_PinState left_val = HAL_GPIO_ReadPin(LEFT_SENSOR_PORT,
	LEFT_SENSOR_PIN);
	GPIO_PinState right_val = HAL_GPIO_ReadPin(RIGHT_SENSOR_PORT,
	RIGHT_SENSOR_PIN);
	GPIO_PinState middle_val = HAL_GPIO_ReadPin(MIDDLE_SENSOR_PORT,
	MIDDLE_SENSOR_PIN);

	if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_SET
			&& middle_val == GPIO_PIN_SET) {
		// Black Black Black - buggy on track so move forwards
//		move_forwards(htim);
		lap_finished = false;
		last_turn_direction = FORWARD;
		if (left_turn_amount < left_veer_decay) {
			left_turn_amount = 0;
		} else {
			left_turn_amount -= left_veer_decay;
			veer_left(htim, left_turn_amount);
		}
		if (right_turn_amount < right_veer_decay) {
			right_turn_amount = 0;
		} else {
			right_turn_amount -= right_veer_decay;
			veer_right(htim, right_turn_amount);
		}
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_SET
			&& middle_val == GPIO_PIN_SET) {
		// White Black Black - buggy is veering left slightly
		left_turn_amount = 0;
		right_turn_amount += right_slight_veer_step;
		if (right_turn_amount > FORWARDS_RIGHT_MOTOR_SPEED) {
			right_turn_amount = FORWARDS_RIGHT_MOTOR_SPEED;
		}
		veer_right(htim, right_turn_amount);
		last_turn_direction = RIGHT;
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_SET
			&& middle_val == GPIO_PIN_RESET) {
		// White White Black - buggy may be veering left strongly or finished a lap
		if (last_turn_direction == RIGHT) {
			left_turn_amount = 0;
			right_turn_amount += right_strong_veer_step;
			if (right_turn_amount > FORWARDS_RIGHT_MOTOR_SPEED) {
				right_turn_amount = FORWARDS_RIGHT_MOTOR_SPEED;
			}
			veer_right(htim, right_turn_amount);
		} else if (last_turn_direction == FORWARD) {
			lap_passed();
		}
	} else if (left_val == GPIO_PIN_RESET && right_val == GPIO_PIN_RESET
			&& middle_val == GPIO_PIN_RESET) {
		// White White White - buggy should have finished a lap - do not reset turning
		lap_passed();
	} else if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_RESET
			&& middle_val == GPIO_PIN_SET) {
		// Black Black White - buggy is veering right slightly
		right_turn_amount = 0;
		left_turn_amount += left_slight_veer_step;
		if (left_turn_amount > FORWARDS_LEFT_MOTOR_SPEED) {
			left_turn_amount = FORWARDS_LEFT_MOTOR_SPEED;
		}
		veer_left(htim, left_turn_amount);
		last_turn_direction = LEFT;
	} else if (left_val == GPIO_PIN_SET && right_val == GPIO_PIN_RESET
			&& middle_val == GPIO_PIN_RESET) {
		// Black White White - buggy may be veering right strongly or finished a lap
		if (last_turn_direction == LEFT) {
			right_turn_amount = 0;
			left_turn_amount += left_strong_veer_step;
			if (left_turn_amount > FORWARDS_LEFT_MOTOR_SPEED) {
				left_turn_amount = FORWARDS_LEFT_MOTOR_SPEED;
			}
			veer_left(htim, left_turn_amount);
		} else if (last_turn_direction == FORWARD) {
			lap_passed();
		}
	}
	taskEXIT_CRITICAL(); // enable interrupts

}

void lap_passed() {
	if (!lap_finished) {
		int currentTick = HAL_GetTick();
		save_lap_time(currentTick - lap_start_tick, lap_ticks_stopped);
		lap_start_tick = currentTick;
		lap_ticks_stopped = 0;
		request_playNote = true;
		lap_finished = true;
		lap_count += 1;
		lap_display = true;
	}
}

void save_lap_time(uint32_t lap_time, uint32_t stopped_time) {
	last_three_lap_times[2] = last_three_lap_times[1];
	last_three_lap_times[1] = last_three_lap_times[0];
	last_three_lap_times[0] = lap_time;

	last_three_lap_speeds[2] = last_three_lap_speeds[1];
	last_three_lap_speeds[1] = last_three_lap_speeds[0];
	last_three_lap_speeds[0] = calculate_lap_speed(lap_time, stopped_time);
}

// 000123 -> 000.123
uint16_t calculate_lap_speed(uint32_t lap_time, uint32_t stopped_time) {
	uint32_t moving_time = lap_time - stopped_time;
	float speed = ((float)moving_time * METERS_PER_SECOND)/(float)lap_time;
	return (uint16_t)(speed*10000.0);
}
