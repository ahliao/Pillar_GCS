// File: FlightControl.cpp
// Brief: Class with altitude and horizontal control functions

#include "inc/FlightControl.h"
#include "inc/common.h"
#include "inc/UART.h"
#include <string.h>

FlightControl::FlightControl()
{
	last_alt = -999;
	last_roll = -999;
	last_pitch = -999;
	last_yaw = -999;
}

void FlightControl::init()
{
	last_alt = -999;
	last_roll = -999;
	last_pitch = -999;
	last_yaw = -999;
	alt_I_term = -999;
}

void FlightControl::calcTime()
{
	time_elapsed = (TCNT0 + (255 - last_time) + 255 * 
			(control_overflow_counter)) * 4e-6;
}

// Altitude Controller
// Input is the relative altitude based on the reference 
void FlightControl::altitudeControl(const float altitude_goal, 
		const TelemetryData& telemetry)
{
	if (last_alt == -999) last_alt = telemetry.uav_alt;
	if (alt_I_term == -999) alt_I_term = 0;
	float Kd = alt_Kd / time_elapsed;
	float Ki = pitch * time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[1];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = altitude_goal - telemetry.uav_alt;
	D_term = telemetry.uav_alt - last_alt;
	I_term += Ki * error;

	// Limit for the I_term
	if (I_term > 5) I_term = 5;
	else if (I_term < -2) I_term = -2;

	// Scale the error by K and adjust the input
	float delta = error * alt_Kp - Kd * D_term + I_term;
	// Limit the delta change
	if (delta > 0.1 && delta < 2) delta = 1;
	else if (delta < -0.2 && delta > -1) delta = 0;
	else if (delta > 3) delta = 3;
	else if (delta < -1) delta = -1;
	input = input + delta;

	// Limit the input 
	if (error < -0.10 && input > 2931) input = input - 1;
	if (error < 0.10 && error > -0.10 && input < 2914) input = 2914;
	if (input < 2720) input = 2720;
	else if (input > 2990) input = 2990;

	//float blah = telemetry.uav_alt;
	float blah = input;
	uint32_t temp; 
	memcpy(&temp, &blah, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);

	// Load new desired PWM into channel 4
	pwm_desired[1] = input;
}

// Pitch controller
// Input is the desired pitch angle (attitude)
void FlightControl::pitchControl(const float pitch_goal,
		const TelemetryData& telemetry)
{
	if (last_pitch == -999) last_pitch = telemetry.uav_pitch;
	float Kd = pitch_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[2];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_pitch - pitch_goal;
	D_term = telemetry.uav_pitch - last_pitch;

	// Scale the error by K and adjust the input
	input = input + error * pitch_Kp - Kd * D_term;

	// Limit the input pitch
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
	if (last_roll == -999) last_roll = telemetry.uav_roll;
	float Kd = roll_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[3];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_roll - roll_goal;
	D_term = telemetry.uav_roll - last_roll;

	// Scale the error by K and adjust the input
	input = input + error * roll_Kp - Kd * D_term;

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
	// TODO: Replace this with universal timer?
	if (last_yaw == -999) last_yaw = telemetry.uav_heading;
	float Kd = yaw_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[0];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	float error = telemetry.uav_heading - yaw_goal;
	D_term = telemetry.uav_heading - last_yaw;

	// Scale the error by K and adjust the input
	float delta = error * yaw_Kp - Kd * D_term;
	if (delta > 1) delta = 1;
	else if (delta < -1) delta = -1;
	input = input + delta;
	
	if (error > 10 && error < -10) input = 2930;

	// Limit the input yaw
	if (input < 2900) input = 2900;
	else if (input > 3000) input = 3000;

	/*float blah = input;//error * yaw_Kp - Kd*D_term;
	//uint32_t blah = input;
	uint32_t temp; 
	memcpy(&temp, &blah, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/

	// Load new desired PWM into channel 4
	pwm_desired[0] = input;
}

void FlightControl::wpControl(const int32_t wp_x, const int32_t wp_y,
		const TelemetryData& telemetry)
{

}

void FlightControl::postCleanup(const TelemetryData& telemetry)
{
	// Store the telemetry data
	last_alt = telemetry.uav_alt;
	last_pitch = telemetry.uav_pitch;
	last_roll = telemetry.uav_roll;
	last_yaw = telemetry.uav_heading;

	// Reset the counters
	last_time = TCNT0;
	control_overflow_counter = 0;
}
