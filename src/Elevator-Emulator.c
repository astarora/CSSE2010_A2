/*
 * Elevator-Emulator.c
 *
 * Main file
 *
 * Authors: Peter Sutton, Ahmed Baig
 * Modified by Jing Wu
 */ 

/* Definitions */

#define F_CPU 8000000L

/* External Library Includes */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h>

/* Internal Library Includes */

#include "display.h"
#include "ledmatrix.h"
#include "buttons.h"
#include "serialio.h"
#include "terminalio.h"
#include "timer0.h"

/* Data Structures */

typedef enum {UNDEF_FLOOR = -1, FLOOR_0=0, FLOOR_1=4, FLOOR_2=8, FLOOR_3=12} ElevatorFloor;

/* Global Variables */
uint32_t time_since_move;
ElevatorFloor current_position;
ElevatorFloor destination;
bool traveller_present = false;
bool traveller_picked = false;


//joystick
bool joystick_mode = false;
int8_t joystick_direction = 0;
uint8_t previous_floor_joystick = 0;

ElevatorFloor traveller_floor = UNDEF_FLOOR;
ElevatorFloor traveller_dest = UNDEF_FLOOR;
uint8_t with_traveller = 0;
uint8_t without_traveller = 0;
uint8_t seven_seg[10] = { 63,6,91,79,102,109,125,7,127,111};
ElevatorFloor last_reached_floor = FLOOR_0; 


uint16_t freq_to_clock_period(uint16_t freq){
	return (1000000UL / freq);
}

uint16_t duty_cycle_to_pulse_width(float duty_cycle, uint16_t clockperiod){
	return (duty_cycle * clockperiod) / 100;
}

//for joystick
uint16_t y_val;
#define JOYSTICK_SLOW_UPPER   600
#define JOYSTICK_FAST_UPPER   800
#define JOYSTICK_SLOW_LOWER   400
#define JOYSTICK_FAST_LOWER   200

//traveller queue
#define max_queue 10
typedef struct {
	ElevatorFloor from;
	ElevatorFloor to;
	uint8_t colour;
}travellers;

traveller_queue [max_queue];
uint8_t queue_head = 0;
uint8_t queue_tail = 0;
uint8_t queue_size = 0;

travellers current_traveller;
bool pick_up = true;






/* Internal Function Declarations */

void initialise_hardware(void);
void start_screen(void);
void start_elevator_emulator(void);
void handle_inputs(void);
void draw_elevator(void);
void draw_floors(void);
void draw_traveller(void);
void SSD_direction (char segment);
void play_tone_500Hz_100ms(void);
void door_animation(void);
void delay_ms_variable(uint16_t ms) {
	while(ms--) {
		_delay_ms(1);  // 每次延迟1ms
	}
}


/* Main */

int main(void) {
	// Setup hardware and call backs. This will turn on 
	// interrupts.
	initialise_hardware();

	// Show the splash screen message. Returns when display is complete
	start_screen();

	// Start elevator controller software
	start_elevator_emulator();

	
}



/* Internal Function Definitions */

/**
 * @brief All hardware initialisation occurs here
 * @arg none
 * @retval none
*/
void initialise_hardware(void) {

	ledmatrix_setup();
	init_button_interrupts();
	// Setup serial port for 19200 baud communication with no echo
	// of incoming characters
	init_serial_stdio(19200,0);

	//Port D
	//S2 for controlling speed
	DDRD &= ~(( 1 << PD2)|(1 << PD3)|(1<<PD5)); // Set S2 and S1 as input
	PORTD |= (1 << PD2)| (1 << PD3)|(1<<PD5); // Enable pull-up resistors on S2 and S1

	DDRD |= (1<<PD6); // Set PD6 as output 

	//joystick
	DDRD &= ~(1 << PD7);
	PORTD |= (1 << PD7);

	//Port C
	//SSD direction indicator set up,output = 1
	DDRC |= (1<<PC0)| (1<<PC1) | (1<<PC2)| (1<<PC3)| (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7); 
	PORTC &= ~((1<<PC0)| (1<<PC1) | (1<<PC2)|(1<<PC3)| (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7)); 
	// Set to off the SSD first then it can on by the order

	//Port A
	//L0 - L3 connect to the C0 - C3, output, DDRC = 1
	DDRA |= (1<< PA0)| (1<<PA1) | (1<<PA2) |(1<<PA3);
	PORTA &= ~((1<< PA0)| (1<<PA1) | (1<<PA2) |(1<<PA3));
	DDRA &= ~(1 << PA4);  // Set PA4 as input for joystick
	PORTA &= ~(1 << PA4); 
	OCR1A = 999;
	TCCR1A = (0 << COM1A1) | (1 << COM1A0) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 <<CS10);
	//set the timer2
	OCR2A = 124;
	TCCR2A = (0 << COM2A1) | (1 << COM2A0) | (1 << WGM11) | (0 << WGM10);
	TCCR2B = (0 << WGM12) | (0 << CS22) | (1 << CS21) | (1 <<CS20);
	
	init_timer0();
	// Set up ADC - AVCC reference, right adjust
	ADMUX = (1<<REFS0);
	// Turn on the ADC (but don't start a conversion yet). Choose a clock
	// divider of 64. (The ADC clock must be somewhere
	// between 50kHz and 200kHz. We will divide our 8MHz clock by 64
	// to give us 125kHz.)
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);

	// Turn on global interrupts
	sei();
}

/**
 * @brief Displays the "EC" start screen with elevator symbol
 * @arg none
 * @retval none
*/

//joystick service mode

void joy_service(void) {//set up ADC
	static uint32_t last_move_time = 0;
	uint32_t now = get_current_time();
	uint8_t current_floor = current_position / 4;
	ADMUX = (1 << REFS0) | (1 << MUX2);
	ADCSRA |= (1 << ADSC);  
	while(ADCSRA & (1<<ADSC)) {
		; /* Wait until conversion finished */
	}
	y_val = ADC; // read the value



	move_terminal_cursor(10, 5);
	printf_P(PSTR("Joystick Y = %d   "), y_val); //debug

	if (y_val > JOYSTICK_FAST_UPPER && current_position < FLOOR_3) {
        if (now - last_move_time > 50) { // 50ms delay for fast
            current_position++;
			traveller_picked = false;
            last_move_time = now;
			joystick_direction = 1;
			if (current_position % 4 == 0) {
			last_reached_floor = current_position;
			}
        }
    }
    // Slow upward
    else if (y_val > JOYSTICK_SLOW_UPPER && y_val <= JOYSTICK_FAST_UPPER && current_position < FLOOR_3) {
        if (now - last_move_time > 200) { // 200ms delay for slow
            current_position++;
			traveller_picked = false;
            last_move_time = now;
			joystick_direction = 1;
			if (current_position % 4 == 0) {
			last_reached_floor = current_position;
			}
        }
		
    }
    // Fast downward
    else if (y_val < JOYSTICK_FAST_LOWER && current_position > FLOOR_0) {
        if (now - last_move_time > 50) {
            current_position--;
			traveller_picked = false;
            last_move_time = now;
			joystick_direction = -1;
			if (current_position % 4 == 0) {
			last_reached_floor = current_position;
			}
        }
		
    }
    // Slow downward
    else if (y_val < JOYSTICK_SLOW_LOWER && y_val >= JOYSTICK_FAST_LOWER && current_position > FLOOR_0) {
        if (now - last_move_time > 200) {
            current_position--;
			traveller_picked = false;
            last_move_time = now;
			joystick_direction = -1;
			if (current_position % 4 == 0) {
			last_reached_floor = current_position;
			}
		}
    }else{
		joystick_direction = 0;
	}

	current_floor = current_position / 4;
	if (current_floor != previous_floor_joystick) {
		if (traveller_picked) { // use traveller picked up instead
			with_traveller++;
		} else {
			without_traveller++;
		}
		previous_floor_joystick = current_floor;
	}
	move_terminal_cursor(10, 18);
	printf_P(PSTR("Number of floors without traveller: %d"), without_traveller);
}


void start_screen(void) {
	// Clear terminal screen and output a message
	clear_terminal();
	move_terminal_cursor(10,10);
	printf_P(PSTR("Elevator Controller"));
	move_terminal_cursor(10,12);
	printf_P(PSTR("CSSE2010/7201 project by Jing WU"));
	move_terminal_cursor(10, 13);
    printf_P(PSTR("Student Number: 48342234"));

	_delay_ms(1000);
	// Show start screen
	start_display();

	// Animation variables
	uint32_t doors_frame_time = 0;
	uint32_t interval_delay = 150;
	uint8_t frame = 0;
	uint8_t doors_opening_closing = 1; // 1 => opening, 0 => closing

	// Wait until a button is pressed, or 's' is pressed on the terminal
	while(1) {

		// Don't worry about this if/else tree. Its purely for animating
		// the elevator doors on the start screen
		if (get_current_time() - doors_frame_time  > interval_delay) {
			start_display_animation(frame);
			doors_frame_time   = get_current_time(); // Reset delay until next movement update
			if (doors_opening_closing) {
				interval_delay = 150;
				frame++;
				if (frame == 1) interval_delay = 2000;
				if (frame == 3) doors_opening_closing = 0;
			} else {
				interval_delay = 150;
				frame--;
				if (frame == 2) interval_delay = 500;
				if (frame == 0) doors_opening_closing = 1;
			}
		}

		// First check for if a 's' is pressed
		// There are two steps to this
		// 1) collect any serial input (if available)
		// 2) check if the input is equal to the character 's'
		char serial_input = -1;
		if (serial_input_available()) {
			serial_input = fgetc(stdin);
		}
		// If the serial input is 's', then exit the start screen
		if (serial_input == 's' || serial_input == 'S') {
			break;
		}
		// Next check for any button presses
		int8_t btn = button_pushed();
		if (btn != NO_BUTTON_PUSHED) {
			break;
		}
	}
}

/**
 * @brief Initialises LED matrix and then starts infinite loop handling elevator
 * @arg none
 * @retval none
*/

//the function for showing the direction by the segment of SSD

void start_elevator_emulator(void) {

	// Clear the serial terminal
	clear_terminal();

	// Initialise Display
	initialise_display();

	// Clear a button push or serial input if any are waiting
	// (The cast to void means the return value is ignored.)
	(void)button_pushed();
	clear_serial_input_buffer();

	// Initialise local variables
	time_since_move = get_current_time();

	// Draw the floors and elevator
	draw_elevator();
	draw_floors();

	current_position = FLOOR_0;
	destination = FLOOR_0;
	uint16_t move_delay_ms = 200;
	uint8_t current_floor = current_position / 4;
	uint8_t previous_floor = current_floor;
	while(true) {

		joystick_mode = (PIND & (1 << PD7));

			if (joystick_mode) {
				joy_service();
				draw_elevator();
				move_terminal_cursor(10, 11);
				printf_P(PSTR("Joystick mode: YES "));//debug
				continue;
			}



		//changing the speed by S2
		if(PIND & (1 << PD5)){
			move_delay_ms = 100;
		}else {
			move_delay_ms = 200;
		}

		// Only update the elevator every 200 ms
		if (joystick_mode) {
			joy_service();
			} else if (get_current_time() - time_since_move > move_delay_ms) {
			
			if (destination > current_position) {
				current_position++;
			} else if (destination < current_position) {
				current_position--;
			}
		

			//terminal display 2
			current_floor = current_position / 4;
			if (current_floor != previous_floor) {
				if (traveller_picked) { // use traveller picked up instead
					with_traveller++;
				} else {
					without_traveller++;
				}
				previous_floor = current_floor;
			}

			if (current_position == FLOOR_0 || current_position == FLOOR_1 ||
				current_position == FLOOR_2 || current_position == FLOOR_3) {
				last_reached_floor = current_position;
			}

			// As we have potentially changed the elevator position, lets redraw it
			draw_elevator();
			if (traveller_present && current_position == traveller_floor) {
    		update_square_colour(4, current_position + 1, EMPTY_SQUARE); // remove traveller
			destination = traveller_dest; // set destination to traveller's destination
			traveller_picked = true;// traveller picked up = true
			play_tone_500Hz_100ms();
			door_animation();
			}

			//traveller movement condition
			if (traveller_present && current_position == traveller_dest && destination == traveller_dest) {
    		traveller_present = false;
			door_animation();
			play_tone_500Hz_100ms();
    		traveller_floor = UNDEF_FLOOR;
			}

			


				
			// print the direction of movement
			move_terminal_cursor(10, 14);
			printf_P(PSTR("Last floor reached: %d  "), current_position / 4);

			move_terminal_cursor(10,15);
			if(destination - current_position >0){
				printf_P(PSTR("Direction: Up       "));
				
			}else if(destination - current_position < 0){
				printf_P(PSTR("Direction: Down      "));
				
			}else{
				printf_P(PSTR("Direction: Stationary"));
				
			}

			move_terminal_cursor(10, 17);
			printf_P(PSTR("Number of floors with traveller: %d"), with_traveller);
			move_terminal_cursor(10, 18);
			printf_P(PSTR("Number of floors without traveller: %d"), without_traveller);


			time_since_move = get_current_time(); // Reset delay until next movement update
		}

		// Handle any button or key inputs
		if (joystick_mode) return;
		handle_inputs();
		// Determine direction and call SSD_direction with appropriate argument

	}
}


/**
 * @brief Draws 4 lines of "FLOOR" coloured pixels
 * @arg none
 * @retval none
*/

// sound effects
void play_tone_pwm(uint16_t freq, uint16_t duration_ms){
	//modify
	float dutycycle = 10; //%
	uint16_t clockperiod = freq_to_clock_period(freq);
	uint16_t pulsewidth = duty_cycle_to_pulse_width(dutycycle, clockperiod);
	
	//set up
	DDRD |= (1<<PD4); //output =1
	TCCR1A = (1 << COM1B1) | (0 <<COM1B0) | (1 <<WGM11) | (1 << WGM10);//clear on compare match
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);//fast PWM & clk/8

	OCR1A = clockperiod - 1; //start to count from 0, so -1
	OCR1B = (pulsewidth > 0) ? (pulsewidth - 1) : 0;
	delay_ms_variable(duration_ms);

	//stop PWM
	TCCR1A = 0;
	TCCR1B = 0;
	PORTD &= ~(1 << PD4);
}


void play_tone_3kHz_50ms() {
	play_tone_pwm(3000, 50);//set freq = 3KHz, duration of an operation:50ms
}
//traveller_present = true


void play_tone_500Hz_100ms() {
	play_tone_pwm(500, 100);//set freq = 500Hz, duration of an operation:100ms
}// traveller_picked and drop of = traveller_present false


// run door animation
void door_animation(void){
	PORTA &= ~((1 << PA0)| (1 << PA3));
	PORTA |= (1 << PA1) | (1 << PA2);
	_delay_ms(400);

	PORTA |= (1 << PA0)| (1 << PA3);
	PORTA &= ~((1 << PA1) | (1 << PA2));
	_delay_ms(400);

	PORTA &= ~((1 << PA0)| (1 << PA3));
	PORTA |= (1 << PA1) | (1 << PA2);
	_delay_ms(400);

	PORTA &= ~((1<< PA0)| (1<<PA1) | (1<<PA2) |(1<<PA3));
}

//traveller queue
//traveller queue up


// If a traveller is dropped off at the floor 
//where the next traveller is waiting (assuming “Travel Queue” is implemented),
//the above animation should only play once. 


void draw_floors(void) {
	for (uint8_t i = 0; i < WIDTH; i++) {
		update_square_colour(i, FLOOR_0, FLOOR);
		update_square_colour(i, FLOOR_1, FLOOR);
		update_square_colour(i, FLOOR_2, FLOOR);
		update_square_colour(i, FLOOR_3, FLOOR);
	}
}

/**
 * @brief Draws the elevator at the current_position
 * @arg none
 * @retval none
*/
void draw_elevator(void) {

	// Store where it used to be with old_position
	static uint8_t old_position; // static variables maintain their value, every time the function is called

	int8_t y = 0; // Height position to draw elevator (i.e. y axis)

	// Clear where the elevator was
	if (old_position > current_position) { // Elevator going down - clear above
		y = old_position + 3;
		} else if (old_position < current_position) { // Elevator going up - clear below
		y = old_position + 1;
	}
	if (y % 4 != 0) { // Do not draw over the floor's LEDs
		update_square_colour(1, y, EMPTY_SQUARE);
		update_square_colour(2, y, EMPTY_SQUARE);
	}
	old_position = current_position;

	// Draw a 2x3 block representing the elevator
	for (uint8_t i = 1; i <= 3; i++) { // 3 is the height of the elevator sprite on the LED matrix
		y = current_position + i; // Adds current floor position to i=1->3 to draw elevator as 3-high block
		if (y % 4 != 0) { // Do not draw on the floor
			update_square_colour(1, y, ELEVATOR);
			update_square_colour(2, y, ELEVATOR); // Elevator is 2 LEDs wide so draw twice
		}
	}
}




/**
 * @brief Reads btn values and serial input and adds a traveller as appropriate
 * @arg none
 * @retval none
*/



void handle_inputs(void) {

	/* ******** START HERE ********
	
	 The following code handles moving the elevator using the buttons on the
	 IO Board. Add code to handle BUTTON2_PUSHED and BUTTON3_PUSHED
	 
	 Here is how the following code works:
	 1. Get btn presses (if any have occurred). Remember that this is
		all handled in the buttons.c/h library.
	 2. Use an if/else tree based on which of the buttons has been
		pressed.
	 3. Set the destination of the elevator to the FLOOR_X corresponding
		with the particular button that was pressed.
	
	*/
	
	
	// We need to check if any button has been pushed

	// button control and moving the traveller#1 & 2 
	uint8_t btn = button_pushed();
	
	if (!traveller_present) {
		uint8_t s0 = (PIND >> PD3) & 1;
		uint8_t s1 = (PIND >> PD2) & 1;
		uint8_t dest = (s1 << 1) | s0;
		ElevatorFloor dest_floor = dest * 4;
		

		if(!traveller_present){
			if (dest ==0){
				traveller_present = false;
				if (btn == BUTTON0_PUSHED) {
					traveller_present = false;
					traveller_picked = false;// set traveller picked up = false
					traveller_floor = FLOOR_0;
					traveller_dest = dest_floor;
					destination = FLOOR_0;
				}else if (btn == BUTTON1_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_1;
					traveller_dest = dest_floor;
					destination = FLOOR_1;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_0);
				}else if (btn == BUTTON2_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_2;
					traveller_dest = dest_floor;
					destination = FLOOR_2;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_0);
				}else if (btn == BUTTON3_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_3;
					traveller_dest = dest_floor;
					destination = FLOOR_3;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_0);
				}
			}else if (dest == 1){
				traveller_present = false;
				if (btn == BUTTON0_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_0;
					traveller_dest = dest_floor;
					destination = FLOOR_0;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_1);
				}else if (btn == BUTTON1_PUSHED) {
					traveller_present = false;
					traveller_picked = false;
					traveller_floor = FLOOR_1;
					traveller_dest = dest_floor;
					destination = FLOOR_1;
				}else if (btn == BUTTON2_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_2;
					traveller_dest = dest_floor;
					destination = FLOOR_2;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_1);
				}else if (btn == BUTTON3_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_3;
					traveller_dest = dest_floor;
					destination = FLOOR_3;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_1);
				}
			}else if (dest == 2){
				if (btn == BUTTON0_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_0;
					traveller_dest = dest_floor;
					destination = FLOOR_0;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_2);
				}else if (btn == BUTTON1_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_1;
					traveller_dest = dest_floor;
					destination = FLOOR_1;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_2);
				}else if (btn == BUTTON2_PUSHED) {
					traveller_present = false;
					traveller_picked = false;
					traveller_floor = FLOOR_2;
					traveller_dest = dest_floor;
					destination = FLOOR_2;

				}else if (btn == BUTTON3_PUSHED) {
					traveller_present = true;
					play_tone_3kHz_50ms();
					traveller_picked = false;
					traveller_floor = FLOOR_3;
					traveller_dest = dest_floor;
					destination = FLOOR_3;
					update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_2);
				}
			}else if (dest == 3){
					if (btn == BUTTON0_PUSHED) {
						traveller_present = true;
						play_tone_3kHz_50ms();
						traveller_picked = false;
						traveller_floor = FLOOR_0;
						traveller_dest = dest_floor;
						destination = FLOOR_0;
						update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_3);
					}else if (btn == BUTTON1_PUSHED) {
						traveller_present = true;
						play_tone_3kHz_50ms();
						traveller_picked = false;
						traveller_floor = FLOOR_1;
						traveller_dest = dest_floor;
						destination = FLOOR_1;
						update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_3);
					}else if (btn == BUTTON2_PUSHED) {
						traveller_present = true;
						play_tone_3kHz_50ms();
						traveller_picked = false;
						traveller_floor = FLOOR_2;
						traveller_dest = dest_floor;
						destination = FLOOR_2;
						update_square_colour(4, traveller_floor + 1, TRAVELLER_TO_3);
					}else if (btn == BUTTON3_PUSHED) {
						traveller_present = false;
						traveller_picked = false;
						traveller_floor = FLOOR_3;
						traveller_dest = dest_floor;
						destination = FLOOR_3;
					}
				
					
			}
		}	
	}

	//control by keyboard
	if(!traveller_present && serial_input_available()){
		char input = fgetc(stdin);
		if (input == '0'){
			destination = FLOOR_0;
		}else if(input == '1'){
			destination = FLOOR_1;
		}else if(input == '2'){
		destination = FLOOR_2;
		}else if(input == '3'){
		destination = FLOOR_3;
		}
	}
}