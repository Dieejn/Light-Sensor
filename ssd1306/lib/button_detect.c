/*
 * BUTTON_DETECT.c
 *
 *  Created on: Dec 25, 2023
 *      Author: vutoandien
 */

#include "button_detect.h"

static state_button_t state_button = state_null;
static uint8_t current_state = 0x7E;
static uint8_t last_state = 0x7E;
static uint8_t deboucing_state = 0x7E;
static uint8_t is_deboucing = 0;
static uint32_t deboucing_timer = 0;
static uint8_t current_input = 0;


void check_state_button(state_button_t *state)
{
	switch ((current_input & 0x7E))
	{
		case 0b01111100:
			*state = state_left;
		break;
		case 0b01111010:
			*state = state_mode;
		break;
		case 0b01110110:
			*state = state_down;
		break;
		case 0b01101110:
			*state = state_up;
		break;
		case 0b01011110:
			*state = state_right;
		break;
		case 0b00111110:
			*state = state_set;
		break;
	}
}

static void BUTTON_Handle(uint8_t state_pin, state_button_t *state)
{
	// detecting
	uint8_t temp_state = state_pin;
	if(temp_state != deboucing_state)
	{
		deboucing_state = temp_state;
		deboucing_timer = HAL_GetTick();
		is_deboucing = 1;
	}

	// debouncing
	if(is_deboucing == 1 && (HAL_GetTick() - deboucing_timer) > 15)
	{
		current_state = deboucing_state;
		is_deboucing = 0;
	}

	if(current_state != last_state)
	{
		switch ((current_input & 0x7E))
		{
			case 0b01111100:
				*state = state_left;
			break;
			case 0b01111010:
				*state = state_mode;
			break;
			case 0b01110110:
				*state = state_down;
			break;
			case 0b01101110:
				*state = state_up;
			break;
			case 0b01011110:
				*state = state_right;
			break;
			case 0b00111110:
				*state = state_set;
			break;
			case 0b01111110:
				*state = state_null;
			break;
		}
		last_state = current_state;
	}
}

state_button_t button_detect(void)
{
	current_input = GPIOA->IDR;
	BUTTON_Handle(current_input & 0x7E, &state_button);
	return state_button;
}
