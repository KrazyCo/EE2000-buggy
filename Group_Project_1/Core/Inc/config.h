/*
 * config.h
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define LEFT_SENSOR_PIN   GPIO_PIN_0
#define LEFT_SENSOR_PORT  GPIOA
#define RIGHT_SENSOR_PIN  GPIO_PIN_1
#define RIGHT_SENSOR_PORT GPIOA
#define MIDDLE_SENSOR_PIN  GPIO_PIN_0
#define MIDDLE_SENSOR_PORT GPIOB

#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_4
#define ECHO_PORT GPIOA

// Base speed out of 999
#define STRAIGHT_SPEED 600

// Change in speed between wheels will cause slight veer, larger difference = larger veer
#define VEER_OUTSIDE_SPEED 650  // Faster wheel
#define VEER_INSIDE_SPEED 550   // Slower wheel

// Recovery turn speed if buggy gets lost
#define RECOVERY_TURN_SPEED 500

#endif /* INC_CONFIG_H_ */
