#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

/********************************************************
 * State Machine for RGB LED
 *
 * States:
 * - Inicio
 * - Red
 * - Blue
 * - Green
 * - White
 *
 *
 *
 *
 *******************************************************/

typedef enum _states_t_{
	ST_INICIO,
	ST_BLUE,
	ST_RED,
	ST_GREEN,
	ST_WHITE
}states_t;


typedef enum _button_t_{
	BT_SW2,
	BT_SW3
}button_t;

void state_ledcolor(void);
void state_machine_init(states_t *ptr_state);
void state_machine (button_t button);

#endif
