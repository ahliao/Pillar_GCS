// File: FlightControl.h
// Brief: Class with altitude and horizontal control functions

#ifndef _FLIGHTCONTROL_H
#define _FLIGHTCONTROL_H

#include "inc/TelemetryData.h"

class FlightControl
{
	public:
		FlightControl();

		// Altitude controller
		// Input is the desired altitude and current telemetry data
		void altitudeControl(const int32_t altitude_goal, 
				const TelemetryData& telemetry);

		// Pitch controller
		// Input is the desired pitch angle (attitude)
		void pitchControl(const int32_t pitch_goal,
				const TelemetryData& telemetry);

		// Roll controller
		// Input is the desired roll angle (attitude)
		void rollControl(const int32_t roll_goal,
				const TelemetryData& telemetry);

		// Yaw controller
		// Input is the desired yaw angle (attitude)
		void yawControl(const int32_t yaw_goal,
				const TelemetryData& telemetry);

		// WP Controller
		void wpControl(const int32_t wp_x, const int32_t wp_y,
				const TelemetryData& telemetry);

	private:
		

};

#endif
