#include <string.h>
#include "inc/MissionControl.h"
#include "inc/common.h"
#include "inc/UART.h"

MissionControl::MissionControl()
{
	// TODO: load in mission from EEPROM or PI
	mission.numofActions = 5;
	mission.actionIndex = 0;
	MissionAction action;
	action.type = ACTION_INIT;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[0] = action;
	action.type = ACTION_TAKEOFF;
	action.altitude = 0.5;
	action.waypointLong = 0;	// Ignore wp for action_takeoff
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[1] = action;
	mission.action = action;
	action.type = ACTION_HOVER;
	action.altitude = 0.5;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 10;
	mission.actions[2] = action;

	action.type = ACTION_WP;
	action.altitude = 0.5;
	// -83.716132, 42.292515 Waypoint 
	// -83.716029, 42.2925050
	// -83.715346, 42.291997 // mole hill
	action.waypointLong = -83.7155783;
	action.waypointLat = 42.2923816;
	action.time = 8;
	mission.actions[1] = action;
	action.type = ACTION_LAND;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[2] = action;
	action.type = ACTION_END;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[3] = action;

	/*action.type = ACTION_WP;
	action.altitude = 0.5;
	action.waypointLong = -83.725463;
	action.waypointLat = 42.2944230;
	action.time = 3;
	mission.actions[1] = action;
	action.type = ACTION_WP;
	action.altitude = 0.5;
	action.waypointLong = -83.725678;
	action.waypointLat = 42.294778;
	action.time = 3;
	mission.actions[2] = action;
	action.type = ACTION_LAND;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[3] = action;
	action.type = ACTION_END;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[4] = action;*/

	// -83.725685, 42.294877	// Starting
	// -83.725581, 42.294688	// Down the field
	// -83.725685, 42.294877	// Starting point (end)

	/*action.type = ACTION_LAND;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[4] = action;
	action.type = ACTION_END;
	action.altitude = 0;
	action.waypointLong = 0;	
	action.waypointLat = 0;
	action.time = 0;
	mission.actions[5] = action;*/

	telemetry.uav_rssi = 0;
	telemetry.uav_linkquality = 0;
	telemetry.uav_failsafe = 0;
	telemetry.uav_arm = 0;
	telemetry.uav_flightmode = 0;
	telemetry.uav_roll = 0;
	telemetry.uav_pitch = 0;
	telemetry.uav_heading = 0;
	telemetry.uav_lat = 0;
	telemetry.uav_lon = 0;
	telemetry.uav_satellites_visible = 0;
	telemetry.uav_fix_type = 0;
	telemetry.uav_gpsheading = 0;
	telemetry.uav_alt = 0;
	telemetry.uav_groundspeed = 0;
	telemetry.uav_bat = 0;
	telemetry.uav_current = 0;
	telemetry.uav_amp = 0;
}

void MissionControl::init()
{
	if (altimeter.init()) {
		//TODO: Handle an error in I2C
		altimeterError = true;
	} else {
		altimeterError = false;
		// Init the altitude controller
		// TODO: set an average
		/*float sum = 0, temp = 0, ref = 0;
		for (int i = 0; i < 3; ) {
			temp = altimeter.getAltitude();
			if (temp <= 100) continue;
			++i;
			sum += temp;
		}
		ref = sum / 3.0;
		altimeterReference = ref;
		flightcontrol.altitudeInit(ref);*/
	}
	flightcontrol.init();

	// TODO: standarize coding style
	yawStart = -999;
	hover_start = -999;
	landing_dest = -9999;
	//latdist = -9999;
	//lngdist = -9999;
}

void MissionControl::runManual()
{
	//uavtalk.read(telemetry);

	if (!altimeterError) {
		float tempalt = altimeter.getAltitude();
		// TODO: Check this
		if (telemetry.uav_alt > 4.4 && tempalt != -999) {
			if (altimeterOffset == 0)
				altimeterOffset = tempalt - telemetry.uav_alt;
			telemetry.uav_alt = tempalt - altimeterOffset;
		}
	}

	if (yawStart == -999 && telemetry.uav_heading != 0) 
		yawStart = telemetry.uav_heading;

	// Handle precontrol calculations
	flightcontrol.calcTime();

	// Run the roll/pitch controllers is input is close to middle
	if (pwm_desired[3] >= 2890 && pwm_desired[3] <= 2930) {
		flightcontrol.rollControl(0.00, telemetry);
	}
	if (pwm_desired[2] >= 2600 && pwm_desired[2] <= 2700) {
		flightcontrol.pitchControl(0.00, telemetry);
	}

	// Run the yaw controller to be fixed yaw
	if (yawStart != -999 && pwm_desired[0] >= 2858 && pwm_desired[0] <= 2948)
		flightcontrol.yawControl(yawStart, telemetry);
}

// Returns a 0 if the current action had no errors
// else returns a 1
uint8_t MissionControl::runMission() 
{
	// If altimeter isn't working, stop

	// Get the new UAVTalk data
	//uavtalk.read(telemetry);

	if (yawStart == -999) yawStart = telemetry.uav_heading;

	/*float lat = telemetry.uav_lat;
	uint32_t temp; 
	memcpy(&temp, &lat, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/

	// TODO:Get the altitude reading
	if (!altimeterError) {
		float tempalt = altimeter.getAltitude();
		// TODO: Check this
		if (telemetry.uav_alt > 4.4 && tempalt != -999) {
			if (altimeterOffset == 0)
				altimeterOffset = tempalt - telemetry.uav_alt;
			telemetry.uav_alt = tempalt - altimeterOffset;
		}
	}

	// Handle precontrol calculations
	flightcontrol.calcTime();

	MissionAction action = mission.actions[mission.actionIndex];
	float errorLng;
	float errorLat;

	switch(action.type) {
		case ACTION_INIT:
			// TODO: Run safety checks

			// Set high thrust for stable takeoff
			pwm_desired[1] = 2600;

			// Go to the takeoff (next step)
			++mission.actionIndex;

			break;
		case ACTION_TAKEOFF:
			// Run the safety checks
			// If problems occurred, return a 1
			//if (runSafetyChecks()) return 1;
			
			// Set the goal altitude and run the altitude controller
			flightcontrol.altitudeControl(action.altitude, telemetry);

			// Set the roll and pitch angles to be 0.00
			flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);

			if (yawStart != -999)
				flightcontrol.yawControl(yawStart, telemetry);

			if (telemetry.uav_alt > action.altitude - 0.25 
					&& telemetry.uav_alt < action.altitude + 0.25)
				++mission.actionIndex;

			break;
		case ACTION_HOVER:
			// keep track of when hover_start is called
			if (hover_start < 0) {
				hover_start = TCNT0;
				hover_overflow_counter = 0;
			}

			// If the hover has lasted long enough, to go next action
			// Multiply by four because each timer count is 4us
			hover_time = (uint16_t)(TCNT0 + (255 - hover_start)) * 0.000004f + 
				hover_overflow_counter * 0.00102f;
			// TODO: Set the time from the action
			if (hover_time >= action.time) {
				hover_start = -999;
				hover_overflow_counter = 0;
				++mission.actionIndex;
			}

			// TODO: Set the goal altitude as the action alt
			flightcontrol.altitudeControl(action.altitude, telemetry);

			// Set the roll and pitch angles to be 0.00
			flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);
			//pwm_desired[3] = 2910;
			//pwm_desired[2] = 2630;

			break;
		case ACTION_WP:
			// If there are enough satellites
			//if (telemetry.uav_satellites_visible >= 4) {
				// Call temp waypoint controller
				/*errorLong = action.waypointLong - telemetry.uav_lon;
				errorLat = action.waypointLat - telemetry.uav_lat;
				controlWaypoint(action.waypointLong, action.waypointLat);*/
			controlWaypoint(action);

				// Call latitude controller
				//moveLat(action);
				// Call longitude controller
				//moveLng(action);
			//flightcontrol.rollControl(10.00, telemetry);
			/*} else {
				// Stay there
				flightcontrol.rollControl(0.00, telemetry);
				flightcontrol.pitchControl(0.00, telemetry);
			}*/
			// Maintain altitude
			flightcontrol.altitudeControl(action.altitude, telemetry);


			// If near the area start timer
			errorLng = action.waypointLong - telemetry.uav_lon;
			errorLat = action.waypointLat - telemetry.uav_lat;
			if (errorLat < 0.00009 && errorLat > -0.00009 &&
					errorLng < 0.00009 && errorLng > -0.00009) {
				++mission.actionIndex;
				/*if (hover_start < 0) {
					hover_start = TCNT0;
					hover_overflow_counter = 0;
				}

				// Multiply by four because each timer count is 4us
				hover_time = (uint16_t)(TCNT0 + (255 - hover_start)) 
					* 0.000004f + hover_overflow_counter * 0.00102f;

				if (hover_time >= action.time) {
					hover_start = -999;
					hover_overflow_counter = 0;
					++mission.actionIndex;
				}*/
			}
			// If the hover has lasted long enough, to go next action

			break;
		case ACTION_LAND:
			if (landing_dest == -9999) landing_dest = telemetry.uav_alt;

			// keep track of when hover_start is called
			if (telemetry.uav_alt - landing_dest < 0.25 && 
					telemetry.uav_alt - landing_dest > -0.25) {
				if (hover_start < 0) {
					hover_start = TCNT0;
					hover_overflow_counter = 0;
				}
			}

			// If the hover has lasted long enough, to go next action
			// Multiply by four because each timer count is 4us
			hover_time = (uint16_t)(TCNT0 + (255 - hover_start)) * 0.000004f + 
				hover_overflow_counter * 0.00102f;
			// TODO: Set the time from the action
			if (hover_time >= 2) {
				hover_start = -999;
				hover_overflow_counter = 0;

				if (landing_dest > 0.0) landing_dest -= 0.1;
				else landing_dest = -8999;
				//++mission.actionIndex;
			}

			// TODO: Set the goal altitude as the action alt
			//if (landing_dest > 0.1)
				flightcontrol.altitudeControl(-100, telemetry);
			//else pwm_desired[1] = 2250;

			// Set the roll and pitch angles to be 0.00
			flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);
			break;
		case ACTION_END:
			pwm_desired[1] = 2100;
			break;
		default:
			// Invalid type
			break;

	}
	// Cleanup counters and store telemetry
	flightcontrol.postCleanup(telemetry);

	return 0;
}

// Temp P controller
// TODO: REMOVE ME
void MissionControl::controlWaypoint(const MissionAction& action)
{
	float errorLong = action.waypointLong - telemetry.uav_lon;
	float errorLat = action.waypointLat - telemetry.uav_lat;

	// Assumes we are facing north
	uint16_t inRoll = 2935;
	//if (telemetry.uav_roll > 0) inRoll = 2930;
	//else if (telemetry.uav_roll < 0) inRoll = 2910;
	uint16_t inPitch = 2660;
	if (wpCounter % 2) {
		if (errorLong > 0.000030) inRoll = 2920;
		else if (errorLong < -0.000030) inRoll = 2950;

		if (errorLat > 0.000030) inPitch = 2680;
		else if (errorLat < -0.000030) inPitch = 2640;

		//if (errorLong > 0.00015 || errorLong < -0.00015) pwm_desired[1] = 2000;
		//if (errorLat > 0.00015 || errorLat < -0.00015) pwm_desired[1] = 2000;

		/*if (errorLong > 0.000050) flightcontrol.rollControl(-3, telemetry);
		else if (errorLong < -0.000050) flightcontrol.rollControl(3, telemetry);

		if (errorLat > 0.000050) flightcontrol.pitchControl(3, telemetry);
		else if (errorLat < -0.000050) flightcontrol.pitchControl(-3, telemetry);*/
	} 
	else {
		//flightcontrol.rollControl(0.00, telemetry);
		//flightcontrol.pitchControl(0.00, telemetry);
		if (errorLong > 0.000030) inRoll = 2945;
		else if (errorLong < -0.000030) inRoll = 2925;

		if (errorLat > 0.000030) inPitch = 2645;
		else if (errorLat < -0.000030) inPitch = 2675;
	}

	pwm_desired[3] = inRoll;
	pwm_desired[2] = inPitch;

	wpCounter++;

	/*float blah = telemetry.uav_lon;//error * yaw_Kp - Kd*D_term;
	//uint32_t blah = input;
	uint32_t temp;
	memcpy(&temp, &blah, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/
}

// Read in new telemetry data from UAVTalk
void MissionControl::updateTelemetry()
{
	uavtalk.read(telemetry);
}

// Returns 0 if no failures were found else returns 1
// TODO: Write any errors to a log file or something
uint8_t MissionControl::runSafetyChecks() 
{
	// Check if number of satellites is at least 7
	if (telemetry.uav_satellites_visible < 7) return 1;

	// Communication check is done in the main.cpp

	return 0;
}

// Called by the ultrasound interrupt to update altitude
void MissionControl::setAltitude(const float alt)
{
	// Don't accept crazy differences
	if (telemetry.uav_alt - alt < 2.0 && telemetry.uav_alt - alt > -2.0)
		telemetry.uav_alt = alt;
	//else telemetry.uav_alt -= 0.01;
}

// Called by barometer to set the altitude offset
void MissionControl::setAltOffset(const float off)
{
	altimeterOffset = off;
}

// ------- Waypoint Functions ------ //
void MissionControl::moveLat(const MissionAction& action) {
	// If this is a starting point, save it
	// Or if the distance is too unreasonable, resave it
	/*if (latdist == -9999 || latdist > 0.01 || latdist < -0.01) 
		latdist = telemetry.uav_lat - action.waypointLat;

	// Current error in latitude
	errorLat = action.waypointLat - telemetry.uav_lat;

	// error normalized to the starting error
	float fracLat = errorLat / latdist;
	float pitchAngle = 0;	// Angle to set the pitch for north/south

	// Set absolute value of the fraction (error keeps the direction)
	if (fracLat < 0) fracLat = fracLat * (-1);

	// Adjust the pitch angle depending on how far the current
	// position is to the desired goal
	if (fracLat <= 0.3 && fracLat > 0.1) {
		// adjust pitch angle to slow down (-3%)
		// TODO: check direction
		if (errorLat > 0) pitchAngle = -5.0;
		else if (errorLat < 0) pitchAngle = 5.0;
	}
	else if (fracLat <= 0.5 && fracLat > 0.3) {

		// adjust throttle to 3 degrees to destination (5%)
		if (errorLat > 0) pitchAngle = 3.0;
		else if (errorLat < 0) pitchAngle = -3.0;
	}
	else if (fracLat < 0.9 && fracLat >0.5) {

		// adjust throttle to 5 degrees to destination
		if (errorLat > 0) pitchAngle = 5.0;
		else if (errorLat < 0) pitchAngle = -5.0;
	}
	else if (fracLat >= 0.9) {

		// adjust throttle to 7 degrees to destination
		if (errorLat > 0) pitchAngle = 7.0;
		else if (errorLat < 0) pitchAngle = -7.0;
	}
	else if (fracLat < 0.1) {
		pitchAngle = 0.0;
	}*/
	//flightcontrol.pitchControl(pitchAngle, telemetry);
}

void MissionControl::moveLng(const MissionAction& action)  {
	// If this is a starting point, save it
	// Or if unreasonable distance
	/*if (lngdist == -9999 || lngdist > 0.01 || lngdist < -0.01) 
		lngdist = telemetry.uav_lon - action.waypointLong;

	// Current error in latitude
	errorLng = action.waypointLong - telemetry.uav_lon;

	// error normalized to the starting error
	float fracLng = errorLng / lngdist;
	float rollAngle = 0;	// Angle to set the pitch for north/south

	// Set absolute value of the fraction (error keeps the direction)
	if (fracLng < 0) fracLng = fracLng * (-1);

	// Adjust the pitch angle depending on how far the current
	// position is to the desired goal
	if (fracLng <= 0.3 && fracLng > 0.1) {
		// adjust pitch angle to slow down (-3%)
		// TODO: check direction
		if (errorLng > 0) rollAngle = -5.0;
		else if (errorLng < 0) rollAngle = 5.0;
	}
	else if (fracLng <= 0.5 && fracLng > 0.3) {

		// adjust throttle to 3 degrees to destination (5%)
		if (errorLng > 0) rollAngle = 3.0;
		else if (errorLng < 0) rollAngle = -3.0;
	}
	else if (fracLng < 0.9 && fracLng >0.5) {

		// adjust throttle to 5 degrees to destination
		if (errorLng > 0) rollAngle = 5.0;
		else if (errorLng < 0) rollAngle = -5.0;
	}
	else if (fracLng >= 0.9) {

		// adjust throttle to 7 degrees to destination
		if (errorLng > 0) rollAngle = 7.0;
		else if (errorLng < 0) rollAngle = -7.0;
	}
	else if (fracLng < 0.1) {
		rollAngle = -3.0;
	}*/
	//errorLng = action.waypointLong - telemetry.uav_lon;
	float error =  action.waypointLong - telemetry.uav_lon;//-83.72555500 - telemetry.uav_lon;
	float rollAngle = 0;	// Angle to set the pitch for north/south
	if (error > 0.0000) rollAngle = 2850;
	//else if (errorLng > 0 && errorLng <= 0.0001) rollAngle = 2800;
	else if (error < 0.0000) rollAngle = 2950;
	//else if (errorLng < 0 && errorLng >= -0.0001) rollAngle = 2980;
	pwm_desired[3] = rollAngle;
	//flightcontrol.rollControl(rollAngle, telemetry);
}
