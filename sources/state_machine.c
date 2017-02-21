/*
 * state_machine.c
 *
 *  Created on: Feb 18, 2017
 *      Author: ernesto
 */


#include "state_machine.h"
#include "stdlib.h"
#include "board.h"
#include "fsl_debug_console.h"

uint8_t blue_sw3 = 0;
uint8_t green_sw2 = 0;
uint8_t green_sw3 = 0;
uint8_t red_sw3 =  0;

static states_t *ctrl_state;

void state_ledcolor(void){
	switch (*ctrl_state){
		case ST_INICIO:
			LED_RED_OFF();
			LED_GREEN_OFF();
			LED_BLUE_OFF();
			break;
		case ST_BLUE:
			LED_RED_OFF();
			LED_GREEN_OFF();
			LED_BLUE_ON();
			break;
		case ST_RED:
			LED_RED_ON();
			LED_GREEN_OFF();
			LED_BLUE_OFF();
			break;
		case ST_GREEN:
			LED_RED_OFF();
			LED_GREEN_ON();
			LED_BLUE_OFF();
			break;
		case ST_WHITE:
			LED_RED_ON();
			LED_GREEN_ON();
			LED_BLUE_ON();
			break;
	}
}

void state_machine_init(states_t *ptr_state){
	ctrl_state = ptr_state;
	*ctrl_state = ST_INICIO;

	state_ledcolor();
}

void state_machine (button_t button){

 	switch (*ctrl_state){
	case ST_BLUE:
		if(button == BT_SW2){
			*ctrl_state = ST_RED;
			blue_sw3 = 0;
		}
		else
			if(blue_sw3++){
				*ctrl_state = ST_GREEN;
				blue_sw3 = 0;
			}
		break;
	case ST_RED:
		if(button == BT_SW3)
			if(++red_sw3 >= 3){
				*ctrl_state = ST_BLUE;
				red_sw3 = 0;
			}
		break;
	case ST_GREEN:
		if(button == BT_SW2){
			if(++green_sw2 >= 2){
				*ctrl_state = ST_BLUE;
				green_sw2 = green_sw3 = 0;
			}
		}
		else
			green_sw3++;

		if(green_sw2 && green_sw3)
			*ctrl_state = ST_WHITE;

		break;
	case ST_INICIO:
			 if(button == BT_SW2)
				 *ctrl_state = ST_BLUE;
			 else
				 *ctrl_state = ST_RED;
			break;

	case ST_WHITE:
		PRINTF("FINISH!\n\r");
		break;

	}

	state_ledcolor();



}


