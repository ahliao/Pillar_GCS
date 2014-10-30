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

void wpControl(const int32_t wp_x, const int32_t wp_y,
		const TelemetryData& telemetry)
{

}
