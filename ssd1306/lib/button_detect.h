/*
 * BUTTON_DECTECT.h
 *
 *  Created on: Dec 25, 2023
 *      Author: vutoandien
 */

#ifndef INC_BUTTON_DETECT_H_
#define INC_BUTTON_DETECT_H_

#include "main.h"


typedef enum
{
    state_null = 0,
    state_left,
    state_mode,
    state_down,
    state_up,
    state_right,
    state_set
} state_button_t;

state_button_t button_detect(void);

void check_state_button(state_button_t *state);
#endif /* INC_BUTTON_DETECT_H_ */
