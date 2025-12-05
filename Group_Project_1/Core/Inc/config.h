/*
 * config.h
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_4
#define ECHO_PORT GPIOA

// Recovery turn speed if buggy gets lost
#define RECOVERY_TURN_SPEED 500

#endif /* INC_CONFIG_H_ */
