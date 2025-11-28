/*
 * line_following.c
 *
 *  Created on: Nov 14, 2025
 *      Author: conor
 */

#ifndef INC_LINE_FOLLOWING_C_
#define INC_LINE_FOLLOWING_C_

#define left_slight_veer_step 10
#define left_strong_veer_step 40

#define right_slight_veer_step 10
#define right_strong_veer_step 40

enum Direction {
	FORWARD, LEFT, RIGHT
};

void line_following_init();
void line_following_loop();
void lap_passed();

#endif /* INC_LINE_FOLLOWING_C_ */
