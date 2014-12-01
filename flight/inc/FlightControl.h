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
		void init();

		void calcTime();

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

		void postCleanup(const TelemetryData& telemetry);

	private:
		// Reference ground altitude
		//float ground_reference;

		// Control constants
		const static float roll_Kp = 0.7;
		const static float roll_Ki = 0;
		const static float roll_Kd = 0.01;

		const static float pitch_Kp = 0.7;
		const static float pitch_Ki = 0;
		const static float pitch_Kd = 0.01;

		const static float yaw_Kp = 0.05;
		const static float yaw_Ki = 0;
		const static float yaw_Kd = 0.0000;

		const static float alt_Kp_rise = 1.20;
		const static float alt_Kp_fall = 0.420;
		const static float alt_Ki = 1.0;
		const static float alt_Kd = 4.30;

		uint16_t last_time;
		float time_elapsed;

		uint16_t hover_ref;

		float delta, Kd, Ki, error;
		float last_alt;
		float last_roll;
		float last_pitch;
		float last_yaw;
		float D_term;
		float alt_I_term;
};

#endif
