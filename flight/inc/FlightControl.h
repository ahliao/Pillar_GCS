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
		void pitchControl(const float pitch_goal,
				const TelemetryData& telemetry);

		// Roll controller
		// Input is the desired roll angle (attitude)
		void rollControl(const float roll_goal,
				const TelemetryData& telemetry);

		// Yaw controller
		// Input is the desired yaw angle (attitude)
		void yawControl(const float yaw_goal,
				const TelemetryData& telemetry);

		// WP Controller
		void wpControl(const int32_t wp_x, const int32_t wp_y,
				const TelemetryData& telemetry);

	private:
		// Controll constants
		const static float roll_k = 1;
		const static float pitch_Kp = 200;
		const static float pitch_Ki = 0;
		const static float pitch_Kd = 0;

		const static uint16_t pwm_max = 255;
		const static uint16_t pwm_min = 0; 

		const static long sample_time = 100;
		unsigned long last_time;

		//float I_term = 0;
		//float last_pitch;
};

#endif
