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
	hover_ref = 0;
}

void FlightControl::calcTime()
{
	time_elapsed = (uint16_t)(TCNT0 + (255 - last_time)) * 0.000004f + 
		control_overflow_counter * 0.00102f;
}

// Altitude Controller
// Input is the relative altitude based on the reference 
void FlightControl::altitudeControl(const float altitude_goal, 
		const TelemetryData& telemetry)
{
	if (last_alt == -999) last_alt = telemetry.uav_alt;
	if (alt_I_term == -999) alt_I_term = 0;
	// Take in the current output (feedback)
	uint16_t input = pwm_desired[1];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	error = altitude_goal - telemetry.uav_alt;
	Kd = alt_Kd / time_elapsed;

	Ki = alt_Ki * time_elapsed;

	D_term = telemetry.uav_alt - last_alt;
	//if (D_term < 0) Kd *= 2.5;
	alt_I_term += Ki * error;

	// Limit for the I_term
	if (error > -0.05 && error < 0.05) alt_I_term = 0;
	else if (alt_I_term > 2) alt_I_term = 2;
	else if (alt_I_term < -1) alt_I_term = -1;

	if (hover_ref == 0 && error < 0.05 && error > -0.05) hover_ref = input;
	//if (D_term < -0.00 && error > 0) D_term *= 1.5;
	if (D_term < 0.009 && D_term > -0.002) D_term = 0;

	if (error < 0.05 && error > -0.05 && input < 2800 
			&& input > 2700 && hover_ref == 0 && D_term == 0)
		hover_ref = input;


	if (error > -0.025 && D_term < 0)  {
		D_term *= 4.3;
		alt_I_term = 0;
	}
	delta = error * alt_Kp_rise - Kd * D_term + alt_I_term;
	/*if (error > 0 && D_term < 0 && delta < 0) {
		delta = 2;
	}*/
	//if (error < 0.02 && hover_ref > 0 && input > hover_ref - 5) delta = -1;
	if (error < 0.05) delta = -1;
	else if (error > 0.2 && delta > 0.1) delta = 2;
	else if (delta > 10 && D_term < 0 && error > 0 && input < 2800) delta = 10;
	else if (error > 0 && delta > 3) delta = 3;
	else if (delta > 0.1 && delta <= 1) delta = 1;
	else if (delta < -1 && input > 2720) delta = -1;
	else delta = 0;
	//else if (delta <= -0.10 && delta > -1) delta = -1;
	input = input + delta;

	// Limit the input 
	//if (error < -0.10 && input > 2930) input = input - 1;
	//if (error > -0.05 && input < 2780) input = 2780;
	//else if (error > 0 && D_term < 0 && input < 2870) input = 2870;
	//else if (error > 0 && input < 2800) input = 2800;
	//else if (error > 0.1 && input < hover_ref-70) input = 2880;
	if (altitude_goal < 0.3 && input < 2400) input = 2400;
	else if (input < 2605) input = 2605;
	else if (error > 0 && input < 2650) input = 2650;
	else if (error < 0.1 && hover_ref > 0 && input > hover_ref + 3) 
		input = hover_ref;
	else if (error < 0.1 && input > 2800) input = 2800;
	
	if (input > 2820) input = 2820;

	//float blah = telemetry.uav_alt;
	/*float blah = pwm_desired[0];//delta;//input;
	uint32_t temp; 
	memcpy(&temp, &blah, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/

	// Load new desired PWM into channel 4
	pwm_desired[1] = input;
}

// Pitch controller
// Input is the desired pitch angle (attitude)
void FlightControl::pitchControl(const float pitch_goal,
		const TelemetryData& telemetry)
{
	if (last_pitch == -999) last_pitch = telemetry.uav_pitch;
	Kd = pitch_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[2];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	error = telemetry.uav_pitch - pitch_goal;
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
	Kd = roll_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[3];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	error = telemetry.uav_roll - roll_goal;
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
	Kd = yaw_Kd / time_elapsed;

	// Take in the current output (feedback)
	uint16_t input = pwm_desired[0];

	// Error is between -360.0 to +360.0
	// but usually between -90.0 to +90.0
	error = telemetry.uav_heading - yaw_goal;
	D_term = telemetry.uav_heading - last_yaw;

	// Scale the error by K and adjust the input
	delta = error * yaw_Kp - Kd * D_term;
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
