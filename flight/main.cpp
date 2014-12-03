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

// PWM Inputs on PORTD
#define INPUT_PWM 2,3,4,5,6
//#define INPUT_PWM 0,1,2,3,4

// Pulse input from ultrasound sensor PORTB 0
#define ULTRASOUND_IN 0

// PWM Variables for control signals
volatile uint8_t pwm_inputs = 0;		// Indicates which inputs are counting
volatile int8_t pwm_outputs = 0;	// Indicates which outputs are on

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

// Ultrasound counter
volatile uint16_t ultrasound_start = 0;
volatile uint8_t ultrasound_count = 0;
volatile float ultraAlt = 0;

// Hover counter (also used in landing)
volatile uint16_t hover_overflow_counter = 0;
volatile uint16_t control_overflow_counter = 0;

MissionControl missionControl;

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

	// Setup 8-bit timer for ultrasound timing
	TCCR0B |= (1 << CS01) | (1 << CS00);	// scale at 64
	TIMSK0 |= (1 << TOIE0);

	// Setup Pin interrupts for the PWM inputs pins
	PCMSK0 |= (1 << PCINT0);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) 
			| (1 << PCINT21) | (1 << PCINT22);
	PCICR |= (1 << PCIE2) | (1 << PCIE0);

	// Initialize UART communication for UAVTalk
	UART::initUART(38400, true);

	// Initialize variables
	// Initialize the desired PWM for testing
	pwm_outputs = -1;
	pwm_desired[0] = 2950;
	pwm_desired[1] = 2000;
	pwm_desired[2] = 2700;
	pwm_desired[3] = 3000;
	pwm_desired[4] = 3000;
	uint16_t sum = 0;
	for (i = 0; i < PWM_CHANNELS; ++i) {
		sum += pwm_desired[i];
		pwm_desired_sums[i] = sum;
	}

	// Mission Control to handle upper-level wp control
	missionControl.init();

	sei();	// Global interrupts on

	while(1) 
	{	
		missionControl.updateTelemetry();

		// Check for a change in the switch (channel 5)
		switch (autopilot_state) {
			case AUTOPILOT_MANUAL:
				// If switch is flipped up, switch to auto mode
				if (pwm_switch_counter > 3300) autopilot_state = AUTOPILOT_AUTO;

				// Run assisted manual control
				if (telemetry_update >= 1) {
					missionControl.runManual();
					telemetry_update = 0;
				}
				PORTB |= (1 << 5);

				break;
			case AUTOPILOT_AUTO:
				// If switch is flipped down, switch to manual mode
				if (pwm_switch_counter < 2500) autopilot_state = AUTOPILOT_MANUAL;

				// Run MissionControlA
				if (telemetry_update >= 3) {
					missionControl.runMission();
					telemetry_update = 0;
				}
				PORTB &= ~(1 << 5);
				// TEST:
				/*pwm_desired[0] = 3000;
				pwm_desired[1] = 3000;
				pwm_desired[2] = 3000;
				pwm_desired[3] = 3000;
				pwm_desired[4] = 3000;*/

				break;
			case AUTOPILOT_EMERGENCY:
				// Handle Emergency mode logic

				pwm_desired[0] = 3000;
				pwm_desired[1] = 1000;
				pwm_desired[2] = 3000;
				pwm_desired[3] = 3000;
				pwm_desired[4] = 3000;	
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
	if (pwm_desired[0] > 0) {
		pwm_outputs = 0x01;
		//PORTC |= (1 << pwm_output_pins[0]);
		OCR1B = 1;
	}

	// Recalculate the PWM timer goal values
	pwm_desired_sums[0] = pwm_desired[0] + 22;
	pwm_desired_sums[1] = pwm_desired_sums[0] + pwm_desired[1] + 8;
	pwm_desired_sums[2] = pwm_desired_sums[1] + pwm_desired[2] + 8;
	pwm_desired_sums[3] = pwm_desired_sums[2] + pwm_desired[3] + 8;
	pwm_desired_sums[4] = pwm_desired_sums[3] + pwm_desired[4] + 8;

	telemetry_update++;	// Update the telemetry every 20ms

	// Communication check watchdog logic

	// If no inputs from receiver, increment watchdog
	// If the watchdog_counter is past 5, assume connection lost
	if(autopilot_state != AUTOPILOT_EMERGENCY && watchdog_counter++ > 3) {
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
	if (pwm_outputs >= 1) {
		if (pwm_outputs >= 6)
			PORTD &= ~(1 << pwm_output_pins[pwm_outputs-2]);
		else if (pwm_outputs > 1)
			PORTC &= ~(1 << pwm_output_pins[pwm_outputs-2]);
		if (pwm_outputs <= PWM_CHANNELS) {
			if (pwm_desired[pwm_outputs-1] > 0)
			{
				OCR1B = pwm_desired_sums[pwm_outputs-1];
				if (pwm_outputs < 5)
					PORTC |= (1 << pwm_output_pins[pwm_outputs-1]);
				else PORTD |= (1 << pwm_output_pins[pwm_outputs-1]);
			}
			++pwm_outputs;
		} else {
			OCR1B = pwm_desired[0];
			pwm_outputs = 0;
		}
	}
}

// Read in the PORTB 0 ultrasound sensor
ISR(PCINT0_vect)
{
	// Only PORTB 0 will ever change
	if (PINB & 1) {
		ultrasound_start = TCNT0;
		ultrasound_count = 0;
	} else {
		uint32_t temp = (TCNT0 + (255 - ultrasound_start) + 255 * (ultrasound_count - 1)) * 4;
		float alt = temp / 1000.0;//147 * 0.0254;//1000.0;	// convert the measurement
		// Only use the ultrasound if less than 4.8 meters
		//if (alt < 4.8) {
			missionControl.setAltitude(alt);
			/*uint32_t serialout; 
			memcpy(&serialout, &alt, sizeof(float));
			UART::writeByte(serialout >> 24);
			UART::writeByte(serialout >> 16);
			UART::writeByte(serialout >> 8);
			UART::writeByte(serialout);*/
		//}
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
				if (autopilot_state == AUTOPILOT_MANUAL) {
					if (pwm_desired[0] < 2890 || pwm_desired[0] > 2930 || temp < 2890 || temp > 2930)
						pwm_desired[0] = (temp + pwm_desired[0]) / 2;
				}
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
					if (pwm_desired[2] < 2600 || pwm_desired[2] > 2700 || temp < 2600 || temp > 2700)
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
					if (pwm_desired[3] < 2860 || pwm_desired[3] > 2960 || temp < 2860 || temp > 2960)
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
}

ISR(TIMER0_OVF_vect)
{
	++ultrasound_count;
	++hover_overflow_counter;
	++control_overflow_counter;
}
