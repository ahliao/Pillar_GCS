// File: FlightControl.h
// Brief: Class with altitude and horizontal control functions

#ifndef _FLIGHTCONTROL_H
#define _FLIGHTCONTROL_H

#include "inc/TelemetryData.h"

class FlightControl
{
	public:
		FlightControl();

		// Sets the reference ground altitude
		void altitudeInit(const float ref);

		// Altitude controller
		// Input is the desired altitude and current telemetry data
		void altitudeControl(const float altitude_goal, 
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
		// Reference ground altitude
		//float ground_reference;

		// Control constants
		const static float roll_Kp = 1.0;
		const static float roll_Ki = 0;
		const static float roll_Kd = 0;

		const static float pitch_Kp = 1.0;
		const static float pitch_Ki = 0;
		const static float pitch_Kd = 0;

		const static float yaw_Kp = 0.1;
		const static float yaw_Ki = 0;
		const static float yaw_Kd = 0;

		const static float alt_Kp = 2.5;
		const static float alt_Ki = 0;
		const static float alt_Kd = 0;

		const static uint16_t pwm_max = 255;
		const static uint16_t pwm_min = 0; 

		const static long sample_time = 100;
		unsigned long last_time;

		//float I_term = 0;
		//float last_pitch;
};

#endif
