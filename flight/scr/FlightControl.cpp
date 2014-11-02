// File: FlightControl.cpp
// Brief: Class with altitude and horizontal control functions

#include "inc/FlightControl.h"
#include "inc/common.h"

FlightControl::FlightControl()
{

}

void FlightControl::altitudeControl(const int32_t altitude_goal, 
		const TelemetryData& telemetry)
{
	// TEST
	pwm_desired[0] = 1000;
}

// Pitch controller
// Input is the desired pitch angle (attitude)
void pitchControl(const int32_t pitch_goal,
		const TelemetryData& telemetry)
{

}

// Roll controller
// Input is the desired roll angle (attitude)
void rollControl(const int32_t roll_goal,
		const TelemetryData& telemetry)
{

}

// Yaw controller
// Input is the desired yaw angle (attitude)
void yawControl(const int32_t yaw_goal,
		const TelemetryData& telemetry)
{

}

void wpControl(const int32_t wp_x, const int32_t wp_y,
		const TelemetryData& telemetry)
{

}
