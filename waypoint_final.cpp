// TO DO: determine PWM levels
// TO DO: check hover code
// TO DO: how to check arrival
using namespace std;
#include <string.h>
#include "inc/MissionControl.h"
#include "inc/common.h"
#include "inc/UART.h"
#include <iostream>

	typedef struct _wayP
	 {
	 	float lat;
	 	float lng;

	 } wayP;

	wayP start;
	wayP end;
	wayP current_loc;
	wayP error;
	wayP dist2dest;

	wayP wayP_array [4];

	int wayP_idx = 0;
	int direction;
	const int roll_PWM = 2970;
	const int pitch_PWM = 2700;
	int overshoot;
	int level;

	static void run_update() {
		current_loc.lng = telemetry.uav_lon;
		current_loc.lat = telemetry.uav_lat;
		error.lat = end.lat - current_loc.lat;
		error.lng = end.lng - current_loc.lng;
	}	

	static int calcPWM(float fracGoal) {

		return (200*fracGoal); // make sure fracGoal is a decimal 

	}

	static void update_idx () {
		if (error.lat < 0.05 && error.lat > -0.05 && error.lng < 0.05 && error.lng > -0.05) {

			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			hover_time = (TCNT0 + (255 - hover_start) +
				(hover_overflow_counter - 1) * 255) * 0.004f;

			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;

				wayP_idx++;
			}

			flightcontrol.altitudeControl(action.altitude, telemetry);
			flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);
		
		}
	}

/*	static float distanceCalc(wayP startPt, wayP endPt){
			
		 dist2dest.lat = endPt.lat - startPt.lat;
		 dist2dest.lng = endPt.lng - startPt.lng;
		
	}*/

	int main () {

		start = wayP_array[wayP_idx];
		end = wayP_array[wayP_idx+1];
		dist2dest.lat = end.lat - start.lat;
		dist2dest.lng = end.lng - start.lng;
		run_update();

		if (wayP_idx == 0) {
			direction = 1;
			moveLat ();
			Lng_correction ();
		}
		else if (wayP_idx == 1) {
			direction = 1;
			moveLng ();
			lat_correction ();
		}
		else if (wayP_idx == 2) {
			direction = -1;
			moveLat ();
			Lng_correction ();
		}
		else if (wayP_idx == 3) {
			direction = -1;
			moveLng ();
			lat_correction ();
		}
		update_idx();


	}

// Facing north, move forward
// Latitidue increases, call pitch
// Negative displacement means overshoot. 

	void moveLat () {
		frac = error.lat/dist2dest.lat; 
		if (frac < 0) {
			overshoot = -1;
			frac = frac * (-1);
		}
		else {
			overshoot = 1;
		}
		if (frac <= 0.3 && frac > 0.1) {
			// adjust throttle to < -3%
			level = calcPWM(-0.03) * direction + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level1 << endl;
		}
		else if (frac <= 0.5 && frac > 0.3) {
	
			// adjust throttle to  5%
			level = calcPWM(0.05) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level2 << endl;
		}
		else if (frac < 0.9 && frac >0.5) {
	
			// adjust throttle to 20%
			level = calcPWM(0.2) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level3 << endl;
		}
		else if (frac >= 0.9) {
	
			// adjust throttle to 40%
			level = calcPWM(0.4) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level4 << endl;
		}

		else if (frac < 0.1) {
			// hover code
			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			hover_time = (TCNT0 + (255 - hover_start) +
				(hover_overflow_counter - 1) * 255) * 0.004f;

			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;
			}

			flightcontrol.altitudeControl(action.altitude, telemetry);
	//		flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);
		}
	
	}

	void moveLng () {
		frac = error.lng/dist2dest.lng; 
		if (frac < 0) {
			overshoot = -1;
			frac = frac * (-1);
		}
		else {
			overshoot = 1;
		}
		if (frac <= 0.3 && frac > 0.1) {
			// adjust throttle to < -3%
			level = calcPWM(-0.03) * direction + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level1 << endl;
		}
		else if (frac <= 0.5 && frac > 0.3) {
	
			// adjust throttle to  5%
			level = calcPWM(0.05) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level2 << endl;
		}
		else if (frac < 0.9 && frac >0.5) {
	
			// adjust throttle to 20%
			level = calcPWM(0.2) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level3 << endl;
		}
		else if (frac >= 0.9) {
	
			// adjust throttle to 40%
			level = calcPWM(0.4) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level4 << endl;
		}

		else if (frac < 0.1) {
			// hover code
			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			hover_time = (TCNT0 + (255 - hover_start) +
				(hover_overflow_counter - 1) * 255) * 0.004f;

			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;
			}

			flightcontrol.altitudeControl(action.altitude, telemetry);
			flightcontrol.rollControl(0.00, telemetry);
		//	flightcontrol.pitchControl(0.00, telemetry);
		}
	
	}

	void Lng_correction () {
		frac = error.lng/dist2dest.lat; 
		if (frac < 0) {
			overshoot = -1;
			frac = frac * (-1);
		}
		else {
			overshoot = 1;
		}
		if (frac <= 0.3 && frac > 0.1) {
			// adjust throttle to < -3%
			level = calcPWM(0.03) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level1 << endl;
		}
		else if (frac <= 0.5 && frac > 0.3) {
	
			// adjust throttle to  5%
			level = calcPWM(0.05) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level2 << endl;
		}
		else if (frac < 0.9 && frac >0.5) {
	
			// adjust throttle to 20%
			level = calcPWM(0.2) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level3 << endl;
		}
		else if (frac >= 0.9) {
	
			// adjust throttle to 40%
			level = calcPWM(0.4) * direction * overshoot + roll_PWM;
			FlightControl::rollControl(level,telemetry);
			//cout <<"Threshold level: "<< level4 << endl;
		}

		else if (frac < 0.1) {
			// hover code
			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			hover_time = (TCNT0 + (255 - hover_start) +
				(hover_overflow_counter - 1) * 255) * 0.004f;

			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;
			}

			flightcontrol.altitudeControl(action.altitude, telemetry);
			flightcontrol.rollControl(0.00, telemetry);
		//	flightcontrol.pitchControl(0.00, telemetry);
		}
	
	}

	void lat_correction () {
		frac = error.lat/dist2dest.lng; 
		if (frac < 0) {
			overshoot = -1;
			frac = frac * (-1);
		}
		else {
			overshoot = 1;
		}
		if (frac <= 0.3 && frac > 0.1) {
			// adjust throttle to < -3%
			level = calcPWM(0.03) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level1 << endl;
		}
		else if (frac <= 0.5 && frac > 0.3) {
	
			// adjust throttle to  5%
			level = calcPWM(0.05) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level2 << endl;
		}
		else if (frac < 0.9 && frac >0.5) {
	
			// adjust throttle to 20%
			level = calcPWM(0.2) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level3 << endl;
		}
		else if (frac >= 0.9) {
	
			// adjust throttle to 40%
			level = calcPWM(0.4) * direction * overshoot + pitch_PWM;
			FlightControl::pitchControl(level,telemetry);
			//cout <<"Threshold level: "<< level4 << endl;
		}

		else if (frac < 0.1) {
			// hover code
			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			hover_time = (TCNT0 + (255 - hover_start) +
				(hover_overflow_counter - 1) * 255) * 0.004f;

			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;
			}

			flightcontrol.altitudeControl(action.altitude, telemetry);
		//	flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);
		}
	
	}





