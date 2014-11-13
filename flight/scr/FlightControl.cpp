// File: FlightControl.cpp
// Brief: Class with altitude and horizontal control functions

#include "inc/FlightControl.h"
#include "inc/common.h"
#include "inc/UART.h"
#include <string.h>

FlightControl::FlightControl()
{

}

void FlightControl::altitudeInit(const float ref)
{
	//ground_reference = ref;
}

// Altitude Controller
// Input is the relative altitude based on the reference 
void FlightControl::altitudeControl(const float altitude_goal, 
		const TelemetryData& telemetry)
{
	// Take in the current output (feedback)
	uint16_t input = pwm_desired[1];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = altitude_goal - telemetry.uav_alt;
	float a = input;//telemetry.uav_alt - ground_reference;
	uint32_t temp; 
	memcpy(&temp, &a, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);
	// Scale the error by K and adjust the input
	float delta = error * alt_Kp;
	// Limit the delta change
	if (delta > 0 && delta < 2) delta = 2;
	else if (delta < 0 && delta > -1) delta = -1;
	else if (delta > 5) delta = 5;
	else if (delta < -5) delta = -5;
	input = input + delta;

	// Limit the input 
	if (input < 2600) input = 2600;
	else if (input > 3050) input = 3050;
	else if (error > -0.1 && error < 0.1) input = 2900;

	// Load new desired PWM into channel 4
	pwm_desired[1] = input;
}

// Pitch controller
// Input is the desired pitch angle (attitude)
void FlightControl::pitchControl(const float pitch_goal,
		const TelemetryData& telemetry)
{
	// Take in the current output (feedback)
	uint16_t input = pwm_desired[2];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_pitch - pitch_goal;

	// Scale the error by K and adjust the input
	input = input + error * pitch_Kp;

	// Limit the input roll
	if (input < 2500) input = 2500;
	else if (input > 2900) input = 2900;

	// Load new desired PWM into channel 4
	pwm_desired[2] = input;
}

// Roll controller
// Input is the desired roll angle (attitude)
void FlightControl::rollControl(const float roll_goal,
		const TelemetryData& telemetry)
{
	// Take in the current output (feedback)
	uint16_t input = pwm_desired[3];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_roll - roll_goal;

	// Scale the error by K and adjust the input
	input = input + error * roll_Kp;

	// Limit the input roll
	if (input < 2770) input = 2770;
	else if (input > 3170) input = 3170;

	// Load new desired PWM into channel 4
	pwm_desired[3] = input;
}

// Yaw controller
// Input is the desired yaw angle (attitude)
void FlightControl::yawControl(const float yaw_goal,
		const TelemetryData& telemetry)
{
	// Take in the current output (feedback)
	uint16_t input = pwm_desired[0];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_heading - yaw_goal;

	// Scale the error by K and adjust the input
	input = input + error * yaw_Kp;

	// Limit the input roll
	if (input < 2800) input = 2800;
	else if (input > 3100) input = 3100;

	// Load new desired PWM into channel 4
	pwm_desired[0] = input;
}

void FlightControl::wpControl(const int32_t wp_x, const int32_t wp_y,
		const TelemetryData& telemetry)
{

}
