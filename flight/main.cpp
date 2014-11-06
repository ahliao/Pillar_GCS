// main.cpp to run the autonomous flight controller on
// the Narwhal for the Pillar CropCopter

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "inc/UART.h"
#include "inc/UAVTalk.h"
#include "inc/MissionControl.h"
#include "inc/common.h"
#include "inc/TelemetryData.h"

// Number of PWM channels (Max is about 9 or 10)
//#define PWM_CHANNELS 5

// PWM Outputs on PORTC
#define OUTPUT_PWM 0,1,2,3,7	// last pin is port d7

// PWM Inputs on PORTB
#define INPUT_PWM 2,3,4,5,6
//#define INPUT_PWM 0,1,2,3,4

// PWM Variables for control signals
volatile uint8_t pwm_inputs = 0;		// Indicates which inputs are counting
volatile uint8_t pwm_outputs = 0;	// Indicates which outputs are on

volatile uint16_t pwm_input_counters[PWM_CHANNELS];
volatile uint16_t pwm_input_starts[PWM_CHANNELS];
volatile uint16_t pwm_desired[PWM_CHANNELS];
volatile uint16_t pwm_desired_sums[PWM_CHANNELS];
uint8_t pwm_output_pins[PWM_CHANNELS] = { OUTPUT_PWM };
uint8_t pwm_input_pins[PWM_CHANNELS] = { INPUT_PWM };

// Pin history for the input change interrupt
uint8_t portdhistory = 0xFF;

// Update telemetry boolean
volatile uint8_t telemetry_update = 0;

// RC watchdog counter
volatile uint8_t watchdog_counter = 0;

volatile uint16_t pwm_switch_counter = 0;

// Autopilot State
typedef enum {
	AUTOPILOT_MANUAL,
	AUTOPILOT_AUTO,	// Temp test state, TODO: Switch to autopilot controller (Mission Planner
	AUTOPILOT_EMERGENCY
} AutopilotState;
volatile AutopilotState autopilot_state = AUTOPILOT_MANUAL;

//uint8_t input_curr = 0;

int main()
{
	uint8_t i;

	// Set PWM outputs and inputs
	for (i = 0; i < PWM_CHANNELS; ++i) {
		DDRD &= ~(1 << pwm_input_pins[i]);
		//DDRB &= ~(1 << pwm_input_pins[i]);
		if (i < 4)
			DDRC |= (1 << pwm_output_pins[i]);
		else DDRD |= (1 << pwm_output_pins[i]);

		// Activate PWM input pull-up resistors
		//PORTB |= (1 << pwm_input_pins[i]);
		PORTD |= (1 << pwm_input_pins[i]);
	}
	//DDRD = 0xFE;
	DDRB |= (1 << 5);

	// Setup 16-bit timer for CTC prescale = 1
	// TODO: OCR1A > OCR1B
	TCCR1B |= (1 << CS11) | (1 << WGM12);
	OCR1A	= 39236;	// every 19.7ms
	OCR1B	= 2000;		// 1ms default setting
	TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);

	// Setup Pin interrupts for the PWM inputs pins
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) 
			| (1 << PCINT21) | (1 << PCINT22);
	PCICR |= (1 << PCIE2) | (1 << PCIE0);

	// Initialize UART communication for UAVTalk
	UART::initUART(38400, true);

	// Mission Control to handle upper-level wp control
	MissionControl missionControl;

	// Initialize variables
	// Initialize the desired PWM for testing
	pwm_desired[0] = 2000;
	pwm_desired[1] = 2000;
	pwm_desired[2] = 2000;
	pwm_desired[3] = 2000;
	pwm_desired[4] = 2000;
	uint16_t sum = 0;
	for (i = 0; i < PWM_CHANNELS; ++i) {
		sum += pwm_desired[i];
		pwm_desired_sums[i] = sum;
	}

	sei();	// Global interrupts on

	while(1) 
	{	
		// Check for a change in the switch (channel 5)
		switch (autopilot_state) {
			case AUTOPILOT_MANUAL:
				// If switch is flipped up, switch to auto mode
				if (pwm_switch_counter > 3300) autopilot_state = AUTOPILOT_AUTO;
				break;
			case AUTOPILOT_AUTO:
				// If switch is flipped down, switch to manual mode
				if (pwm_switch_counter < 2500) autopilot_state = AUTOPILOT_MANUAL;

				// Run MissionControl
				missionControl.runMission();
				// TEST:
				/*pwm_desired[0] = 3000;
				pwm_desired[1] = 3000;
				pwm_desired[2] = 3000;
				pwm_desired[3] = 3000;
				pwm_desired[4] = 3000;*/

				break;
			case AUTOPILOT_EMERGENCY:
				// Handle Emergency mode logic
				if (pwm_switch_counter > 3300) autopilot_state = AUTOPILOT_AUTO;
				else if (pwm_switch_counter < 2500 && pwm_switch_counter > 1000) autopilot_state = AUTOPILOT_MANUAL;
				break;
			default:
				// Error occurred; Go into emergency
				autopilot_state = AUTOPILOT_EMERGENCY;
		}
	}

	return 0;
}

// Timer Interrupt for A
ISR(TIMER1_COMPA_vect)
{
	// Increment the 19.7ms PWM counter
	pwm_outputs = 0x01;
	if (pwm_desired[0] > 0)
		PORTC |= (1 << pwm_output_pins[0]);

	// Recalculate the PWM timer goal values
	pwm_desired_sums[0] = pwm_desired[0];
	pwm_desired_sums[1] = pwm_desired_sums[0] + pwm_desired[1] + 12;
	pwm_desired_sums[2] = pwm_desired_sums[1] + pwm_desired[2] + 12;
	pwm_desired_sums[3] = pwm_desired_sums[2] + pwm_desired[3] + 12;
	pwm_desired_sums[4] = pwm_desired_sums[3] + pwm_desired[4] + 12;

	telemetry_update++;	// Update the telemetry every 20ms

	// Communication check watchdog logic

	// If no inputs from receiver, increment watchdog
	// If the watchdog_counter is past 5, assume connection lost
	if(autopilot_state != AUTOPILOT_EMERGENCY && ++watchdog_counter > 5) {
		autopilot_state = AUTOPILOT_EMERGENCY;
		// Shut down everything
		pwm_switch_counter = 0;
		pwm_desired[0] = 3000;
		pwm_desired[1] = 1000;
		pwm_desired[2] = 3000;
		pwm_desired[3] = 3000;
		pwm_desired[4] = 3000;	
	}
}

// Timer 1 Interrupt for B
// Handles the individual channel PWM lengths
ISR(TIMER1_COMPB_vect)
{
	if (pwm_outputs > 0) {
		if (pwm_outputs == 5)
			PORTD &= ~(1 << pwm_output_pins[pwm_outputs-1]);
		else
			PORTC &= ~(1 << pwm_output_pins[pwm_outputs-1]);
		if (pwm_outputs < PWM_CHANNELS) {
			OCR1B = pwm_desired_sums[pwm_outputs];
			if (pwm_desired[pwm_outputs] > 0)
			{
				if (pwm_outputs < 4)
					PORTC |= (1 << pwm_output_pins[pwm_outputs]);
				else PORTD |= (1 << pwm_output_pins[pwm_outputs]);
			}
			++pwm_outputs;
		} else {
			OCR1B = pwm_desired_sums[0];
			pwm_outputs = 0;
		}
	}
}

ISR(PCINT2_vect)
{
	uint8_t changedbits;
	changedbits = PIND ^ portdhistory;
	portdhistory = PIND;

	// Reset the watchdog
	watchdog_counter = 0;

	// TODO: Check only the current and next channel (adjencent)
	//uint8_t input_curr = 0;
	int32_t temp = 0;

	if (changedbits & (1 << 2)) {
		if (PIND & (1 << 2)) {
			// Pin went up; log the time
			pwm_input_starts[0] = TCNT1;
		} else if (!(PIND & (1 << 2))) {
			// Pin went down; store the delta
			temp = TCNT1 - pwm_input_starts[0];
			if (TCNT1 < pwm_input_starts[0]) {
				temp = TCNT1 + OCR1A - pwm_input_starts[0];
			}
			// Make sure the difference is a positive and in a reasonable range
			if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
			{
				// Set PWM output to the delta time found
				if (autopilot_state == AUTOPILOT_MANUAL)
					pwm_desired[0] = (temp + pwm_desired[0]) / 2;
			}
		}
	}
	if (changedbits & (1 << 3)) {
		if (PIND & (1 << 3)) {
			// Pin went up; log the time
			pwm_input_starts[1] = TCNT1;
		} else if (!(PIND & (1 << 3))) {
			// Pin went down; store the delta
			temp = TCNT1 - pwm_input_starts[1];
			if (TCNT1 < pwm_input_starts[1]) {
				temp = TCNT1 + OCR1A - pwm_input_starts[1];
			}
			// Make sure the difference is a positive and in a reasonable range
			if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
			{
				// Set PWM output to the delta time found
				if (autopilot_state == AUTOPILOT_MANUAL)
					pwm_desired[1] = (temp + pwm_desired[1]) / 2;
			}
		}
	}
	if (changedbits & (1 << 4)) {
		if (PIND & (1 << 4)) {
			// Pin went up; log the time
			pwm_input_starts[2] = TCNT1;
		} else if (!(PIND & (1 << 4))) {
			// Pin went down; store the delta
			temp = TCNT1 - pwm_input_starts[2];
			if (TCNT1 < pwm_input_starts[2]) {
				temp = TCNT1 + OCR1A - pwm_input_starts[2];
			}
			// Make sure the difference is a positive and in a reasonable range
			if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
			{
				// Set PWM output to the delta time found
				if (autopilot_state == AUTOPILOT_MANUAL)
					pwm_desired[2] = (temp + pwm_desired[2]) / 2;
			}
		}
	}
	if (changedbits & (1 << 5)) {
		if (PIND & (1 << 5)) {
			// Pin went up; log the time
			pwm_input_starts[3] = TCNT1;
		} else if (!(PIND & (1 << 5))) {
			// Pin went down; store the delta
			temp = TCNT1 - pwm_input_starts[3];
			if (TCNT1 < pwm_input_starts[3]) {
				temp = TCNT1 + OCR1A - pwm_input_starts[3];
			}
			// Make sure the difference is a positive and in a reasonable range
			if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
			{
				// Set PWM output to the delta time found
				if (autopilot_state == AUTOPILOT_MANUAL)
					pwm_desired[3] = (temp + pwm_desired[3]) / 2;
			}
		}
	}
	if (changedbits & (1 << 6)) {
		if (PIND & (1 << 6)) {
			// Pin went up; log the time
			pwm_input_starts[4] = TCNT1;
		} else if (!(PIND & (1 << 6))) {
			// Pin went down; store the delta
			temp = TCNT1 - pwm_input_starts[4];
			if (TCNT1 < pwm_input_starts[4]) {
				temp = TCNT1 + OCR1A - pwm_input_starts[4];
			}
			// Make sure the difference is a positive and in a reasonable range
			if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
			{
				// Set PWM output to the delta time found
				if (autopilot_state == AUTOPILOT_MANUAL)
					pwm_desired[4] = (temp + pwm_desired[4]) / 2;
				// Always track the switch output
				pwm_switch_counter = temp;
			}
		}
	}
	
	
	// Only read inputs if in manual mode (TODO: more flexible?)
	/*if (autopilot_state != AUTOPILOT_MANUAL) input_curr = 4;
	for (; input_curr < PWM_CHANNELS; ++input_curr) {
		// First see if this pin changed, else save from doing two if's
		if (changedbits & (1 << (input_curr + 2)))
		{
			if (PIND & (1 << (input_curr + 2))) {
				// Pin went up; log the time
				pwm_input_starts[input_curr] = TCNT1;
			} else if (!(PIND & (1 << (input_curr + 2)))) {
				// Pin went down; store the delta
				temp = TCNT1 - pwm_input_starts[input_curr];
				if (TCNT1 < pwm_input_starts[input_curr]) {
					temp = TCNT1 + OCR1A - pwm_input_starts[input_curr];
				}
				// Make sure the difference is a positive and in a reasonable range
				if (temp > 0 && temp < 4500) //pwm_desired[input_curr] = (TCNT1 - pwm_input_starts[input_curr]);
				{
					// Set PWM output to the delta time found
					pwm_desired[input_curr] = temp;
					// Always track the switch output
					if (input_curr == 4) pwm_switch_counter = temp;
				}

				// Check if communication is working again by checking the switch
				if (autopilot_state == AUTOPILOT_EMERGENCY && input_curr == 4) {
					if (pwm_desired[4] > 3300) autopilot_state = AUTOPILOT_AUTO;
					else if (pwm_desired[4] > 1000) autopilot_state = AUTOPILOT_MANUAL;
				}
			}
		}
	}*/
}
