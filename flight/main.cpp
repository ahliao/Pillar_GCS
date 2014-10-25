// main.cpp to run the autonomous flight controller on
// the Narwhal for the Pillar CropCopter

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "inc/UART.h"
#include "inc/UAVTalk.h"

// SERVO PORTC
#define SERVO_PIN 3

// LED0 PORTD
#define LED0_PIN 5
#define HEARTBEAT 6

// Number of PWM channels (Max is about 9 or 10)
#define PWM_CHANNELS 5

// PWM Outputs on PORTC
#define OUTPUT_PWM 0,1,2,3,4

// PWM Inputs on PORTB
#define INPUT_PWM 0,1,2,3,4

// PWM Variables for control signals
volatile uint8_t pwm_inputs = 0;		// Indicates which inputs are counting
volatile uint8_t pwm_outputs = 0;	// Indicates which outputs are on

volatile uint16_t pwm_input_counters[PWM_CHANNELS];
volatile uint16_t pwm_desired[PWM_CHANNELS];
volatile uint16_t pwm_desired_sums[PWM_CHANNELS];
uint8_t pwm_output_pins[PWM_CHANNELS] = { OUTPUT_PWM };
uint8_t pwm_input_pins[PWM_CHANNELS] = { INPUT_PWM };

// Pin history for the input change interrupt
uint8_t portbhistory = 0xFF;

// Update telemetry boolean
volatile bool telemetry_update = 0;

// RC watchdog counter
volatile uint8_t watchdog_counter = 0;

// Autopilot State
volatile uint8_t auto_state = 0;

int main()
{
	uint8_t i;

	// Set LED output
	//DDRD |= (1 << LED0_PIN) | (1 << HEARTBEAT);

	// Set PWM outputs and inputs
	for (i = 0; i < PWM_CHANNELS; ++i) {
		DDRB &= ~(1 << pwm_input_pins[i]);
		DDRC |= (1 << pwm_output_pins[i]);

		// Activate PWM input pull-up resistors
		PORTB |= (1 << pwm_input_pins[i]);
	}
	DDRD = 0xFE;
	DDRB |= (1 << 5);

	// Setup 16-bit timer for CTC prescale = 1
	// TODO: OCR1A > OCR1B
	TCCR1B |= (1 << CS11) | (1 << WGM12);
	OCR1A	= 40000;	// every 20ms
	OCR1B	= 2000;		// 10us seems to be the best resolution we can get
	TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);

	// Setup 8-bit timer for CTC prescale = 1
	TCCR0A |= (1 << WGM01);
	OCR0A = 160;	// Every 5us
	TCCR0B |= (1 << CS00);	
	TIMSK0 = (1 << OCIE0A);

	// Setup Pin interrupts for the PWM inputs pins
	// TODO: Make this more flexible (Take in the MACROS)
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) 
			| (1 << PCINT3) | (1 << PCINT4);
	//PCMSK0 |= (1 << PCINT0);
	PCICR |= (1 << PCIE0);

	sei();	// Global interrupts on

	// UAVTalk object to read the UAVObjects
	UAVTalk uavtalk;	// This also calls initUART()
	UART::initUART(38400, true);

	// Initialize variables
	// Initialize the desired PWM for testing
	pwm_desired[0] = 2000;
	pwm_desired[1] = 2500;
	pwm_desired[2] = 3000;
	pwm_desired[3] = 3500;
	pwm_desired[4] = 4000;
	uint16_t sum = 0;
	for (i = 0; i < PWM_CHANNELS; ++i) {
		sum += pwm_desired[i];
		pwm_desired_sums[i] = sum;
	}

	uint8_t data = 0;
	while(1) 
	{	
		// TODO: Add a check on uav incoming data limits
		// Telemetry is updated every 20ms (can be faster)
		// For testing, forward uav_roll to UART
		// TODO: Check if data is being received
		// else the read will hang (check newData)
		/*if (telemetry_update) {
			if(uavtalk.read()) {
				uint32_t temp; 
				memcpy(&temp, &uavtalk.uav_roll, sizeof(float));
				UART::writeByte(temp >> 24);
				UART::writeByte(temp >> 16);
				UART::writeByte(temp >> 8);
				UART::writeByte(temp);
			}
			telemetry_update = 0;	
		}*/

		// TEST: Check the flight mode switch 
		// Switch the LED
		if (pwm_desired[4] < 2000) {
			//PORTB |= (1 << 5);	
		} else {
			//PORTB &= ~(1 << 5);
		}

		// TEST: Both sticks to the right
		if (pwm_desired[0] < 2350 && pwm_desired[3] < 2250) {
			PORTB |= (1 << 5);	
		} else {
			PORTB &= ~(1 << 5);
		}
	}

	return 0;
}

// Timer Interrupt for A
ISR(TIMER1_COMPA_vect)
{
	// Increment the 20ms PWM counter
	pwm_outputs = 0x01;
	if (pwm_desired[0] > 0 && auto_state != 1)
		PORTC |= (1 << pwm_output_pins[0]);
	uint16_t sum = 0;
	for (uint8_t i = 0; i < PWM_CHANNELS; ++i) {
		sum += pwm_desired[i];
		pwm_desired_sums[i] = sum;
	}
	telemetry_update = 1;	// Update the telemetry every 20ms

	// Check state
	// If lost connection
	if (auto_state == 1) {
		for (uint8_t i = 0; i < PWM_CHANNELS; ++i) {
			// TODO: For now just zero everything
			pwm_desired[i] = 0;
		}
	} else if (auto_state == 0) {
		/*pwm_desired[0] = 2000;
		pwm_desired[1] = 2500;
		pwm_desired[2] = 3000;
		pwm_desired[3] = 3500;
		pwm_desired[4] = 4000;*/
	}

	// If no inputs from receiver, increment watchdog
	// If the watchdog_counter is past 5, assume connection lost
	if(++watchdog_counter > 5) {
		auto_state = 1;
		pwm_inputs = 0;
		// TODO: Look into looping
		pwm_input_counters[0] = 0;
		pwm_input_counters[1] = 0;
		pwm_input_counters[2] = 0;
		pwm_input_counters[3] = 0;
		pwm_input_counters[4] = 0;
		PORTC &= ~(1 << 0);
		PORTC &= ~(1 << 1);
		PORTC &= ~(1 << 2);
		PORTC &= ~(1 << 3);
		PORTC &= ~(1 << 4);
	}
}

// Timer 1 Interrupt for B
ISR(TIMER1_COMPB_vect)
{
	if (pwm_outputs > 0) {
		PORTC &= ~(1 << pwm_output_pins[pwm_outputs-1]);
		if (pwm_outputs < PWM_CHANNELS) {
			OCR1B = pwm_desired_sums[pwm_outputs];
			if (pwm_desired[pwm_outputs] > 0)
				PORTC |= (1 << pwm_output_pins[pwm_outputs]);
			++pwm_outputs;
		} else {
			OCR1B = pwm_desired_sums[0];
			pwm_outputs = 0;
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	// Loops seem to suck in interrupts
	if (pwm_inputs & (1 << 0))
		++pwm_input_counters[0];
	if (pwm_inputs & (1 << 1))
		++pwm_input_counters[1];
	if (pwm_inputs & (1 << 2))
		++pwm_input_counters[2];
	if (pwm_inputs & (1 << 3))
		++pwm_input_counters[3];
	if (pwm_inputs & (1 << 4))
		++pwm_input_counters[4];

	// NOTE: Can't use volatile loop variables
	/*for (uint8_t i = 0; i < PWM_CHANNELS; ++i) {
		if (pwm_inputs & (1 << i)) 
			++pwm_input_counters[i];
	}*/
}

// Pin change interrupt for PCINT7..0
ISR(PCINT0_vect)
{
	// TODO: See if loops could work 
	// Loops seem to not work well in interrupts
	uint8_t changedbits;
	changedbits = PINB ^ portbhistory;
	portbhistory = PINB;

	watchdog_counter = 0;
	auto_state = 0;

	for (uint8_t i = 0; i < PWM_CHANNELS; ++i) {
		if (changedbits & (1 << i))
		{
			// PCINT0 changed
			// If PWM pin is on, start the counter
			if (PINB & (1 << i)) {
				pwm_inputs |= (1 << i);
			}

			// Detect when the inputs go low
			else if (pwm_inputs & (1 << i)) {
				if (!(PINB & (1 << i))) {
					// Stop the PWM counter
					pwm_inputs &= ~(1 << i);

					// Update the desired signals
					// Read times are usually about 10-20us less than actual
					pwm_input_counters[i] += 3;	// Fix the slight error
					pwm_desired[i] = pwm_input_counters[i]*20;	// Multiple by 20 scale
					pwm_input_counters[i] = 0;
				}
			}
		}
	}	

	/*if (changedbits & (1 << 0))
	{
		// PCINT0 changed
		// If PWM pin is on, start the counter
		if (PINB & (1 << 0)) {
			pwm_inputs |= (1 << 0);
		}

		// Detect when the inputs go low
		else if (pwm_inputs & (1 << 0)) {
			if (!(PINB & (1 << 0))) {
				// Stop the PWM counter
				pwm_inputs &= ~(1 << 0);

				// Update the desired signals
				// Read times are usually about 10-20us less than actual
				++pwm_input_counters[0];	// Fix the slight error
				pwm_desired[0] = pwm_input_counters[0]*20;	// Multiple by 20 scale
				pwm_input_counters[0] = 0;
			}
		}
	}
	if (changedbits & (1 << 1))
	{
		// PCINT1 changed
		// If PWM pin is on, start the counter
		if (PINB & (1 << 1)) {
			pwm_inputs |= (1 << 1);
		}
		else if (pwm_inputs & (1 << 1)) {
			if (!(PINB & (1 << 1))) {
				// Stop the PWM counter
				pwm_inputs &= ~(1 << 1);

				// Update the desired signals
				// Read times are usually about 10-20us less than actual
				++pwm_input_counters[1];	// Fix the slight error
				pwm_desired[1] = pwm_input_counters[1]*20;	// Multiple by 20 scale
				pwm_input_counters[1] = 0;
			}
		}
	}
	if (changedbits & (1 << 2))
	{
		// PCINT2 changed
		// If PWM pin is on, start the counter
		if (PINB & (1 << 2)) {
			pwm_inputs |= (1 << 2);
		}
		else if (pwm_inputs & (1 << 2)) {
			if (!(PINB & (1 << 2))) {
				// Stop the PWM counter
				pwm_inputs &= ~(1 << 2);

				// Read times are usually about 10-20us less than actual
				++pwm_input_counters[2];	// Fix the slight error
				pwm_desired[2] = pwm_input_counters[2]*20;	// Multiple by 20 scale
				pwm_input_counters[2] = 0;
			}
		}
	}
	if (changedbits & (1 << 3))
	{
		// PCINT3 changed
		// If PWM pin is on, start the counter
		if (PINB & (1 << 3)) {
			pwm_inputs |= (1 << 3);
		}
		else if (pwm_inputs & (1 << 3)) {
			if (!(PINB & (1 << 3))) {
				// Stop the PWM counter
				pwm_inputs &= ~(1 << 3);

				// Read times are usually about 10-20us less than actual
				++pwm_input_counters[3];	// Fix the slight error
				pwm_desired[3] = pwm_input_counters[3]*20;	// Multiple by 20 scale
				pwm_input_counters[3] = 0;
			}
		}
	}
	if (changedbits & (1 << 4))
	{
		watchdog_counter = 0;

		// PCINT4 changed
		if (PINB & (1 << 4)) {
			pwm_inputs |= (1 << 4);
		}

		else if (pwm_inputs & (1 << 4)) {
			if (!(PINB & (1 << 4))) {
				// Stop the PWM counter
				pwm_inputs &= ~(1 << 4);

				// Read times are usually about 10-20us less than actual
				++pwm_input_counters[4];	// Fix the slight error
				pwm_desired[4] = pwm_input_counters[4]*20;	// Multiple by 20 scale
				pwm_input_counters[4] = 0;
			}
		}
	}*/
}
