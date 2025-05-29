/*
 * timer0.c
 *
 * Author: Peter Sutton
 *
 * We setup timer0 to generate an interrupt every 1ms
 * We update a global clock tick variable - whose value
 * can be retrieved using the get_clock_ticks() function.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "timer0.h"

/* Our internal clock tick count - incremented every 
 * millisecond. Will overflow every ~49 days. */
static volatile uint32_t clockTicks;
typedef enum {UNDEF_FLOOR = -1, FLOOR_0=0, FLOOR_1=4, FLOOR_2=8, FLOOR_3=12} ElevatorFloor;
extern ElevatorFloor last_reached_floor;
extern ElevatorFloor current_position;
extern ElevatorFloor destination;
extern uint8_t seven_seg[10];
extern bool joystick_mode;
extern int8_t joystick_direction;
extern int8_t joystick_direction;
#define SEG_DP (1 << PC7);

/* Set up timer 0 to generate an interrupt every 1ms.
 * We will divide the clock by 64 and count up to 124.
 * We will therefore get an interrupt every 64 x 125
 * clock cycles, i.e. every 1 milliseconds with an 8MHz
 * clock. 
 * The counter will be reset to 0 when it reaches it's
 * output compare value.
 */
void init_timer0(void) {
	/* Reset clock tick count. L indicates a long (32 bit)
	 * constant. 
	 */
	clockTicks = 0L;
	
	/* Clear the timer */
	TCNT0 = 0;

	/* Set the output compare value to be 124 */
	OCR0A = 124;
	
	/* Set the timer to clear on compare match (CTC mode)
	 * and to divide the clock by 64. This starts the timer
	 * running.
	 */
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS01)|(1<<CS00);

	/* Enable an interrupt on output compare match. 
	 * Note that interrupts have to be enabled globally
	 * before the interrupts will fire.
	 */
	TIMSK0 |= (1<<OCIE0A);
	
	/* Make sure the interrupt flag is cleared by writing a 
	 * 1 to it.
	 */
	TIFR0 &= (1<<OCF0A);
}


uint32_t get_current_time(void) {
	uint32_t returnValue;

	/* Disable interrupts so we can be sure that the interrupt
	 * doesn't fire when we've copied just a couple of bytes
	 * of the value. Interrupts are re-enabled if they were
	 * enabled at the start.
	 */
	uint8_t interruptsOn = bit_is_set(SREG, SREG_I);
	cli();
	returnValue = clockTicks;
	if(interruptsOn) {
		sei();
	}
	return returnValue;
}

ISR(TIMER0_COMPA_vect) {
	/* Increment our clock tick count */
	clockTicks++;

	static uint8_t digit = 0;

	PORTC = 0;
	PORTD &= ~(1 << PD6);
	if (digit == 0){
		PORTD &= ~(1 << PD6);
		uint8_t floor = last_reached_floor / 4;
		uint8_t seg = seven_seg[floor];
		if (current_position != FLOOR_0 && current_position != FLOOR_1 &&
			current_position != FLOOR_2 && current_position != FLOOR_3) {
			seg |= SEG_DP;
		}
		PORTC = seg;
	} else {
		PORTD |= (1 << PD6); 
		uint8_t seg = 0;


		if (joystick_mode) {
			if (joystick_direction > 0) {
				seg = (1 << PC0);
			} else if (joystick_direction < 0) {
				seg = (1 << PC3);
			} else {
				seg = (1 << PC6);
			}
		} else {
			if (destination > current_position) {
				seg = (1 << PC0); // segment a
			} else if (destination < current_position) {
				seg = (1 << PC3); // segment d
			} else {
				seg = (1 << PC6); // segment g
			}
		}

		PORTC = seg;
	}

	digit = 1 - digit;
}



