#include <string.h>
#include "inc/MissionControl.h"
#include "inc/common.h"
#include "inc/UART.h"

MissionControl::MissionControl()
{
	// TODO: load in mission from EEPROM or PI
	mission.numofActions = 3;
	mission.actionIndex = 0;
	MissionAction action;
	action.type = ACTION_TAKEOFF;
	action.altitude = 20;
	action.waypointX = 0;	// Ignore wp for action_takeoff
	action.waypointY = 0;
	mission.actions[0] = action;
	mission.action = action;
	action.type = ACTION_HOVER;
	action.altitude = 20;
	action.waypointX = 0;	// Ignore wp for action_takeoff
	action.waypointY = 0;
	mission.actions[1] = action;
	action.type = ACTION_LAND;
	action.altitude = 0;
	action.waypointX = 0;	// Ignore wp for action_takeoff
	action.waypointY = 0;
	mission.actions[2] = action;

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

	yawStart = -999;
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
}

void MissionControl::runManual()
{
	uavtalk.read(telemetry);

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

	uint32_t input = pwm_desired[1];
	//float yaw = telemetry.uav_heading - yawStart;
	uint32_t temp; 
	memcpy(&temp, &input, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);

	// Run the roll/pitch controllers is input is close to middle
	if (pwm_desired[3] >= 2920 && pwm_desired[3] <= 3020) {
		flightcontrol.rollControl(0.00, telemetry);
	}
	if (pwm_desired[2] >= 2650 && pwm_desired[2] <= 2750) {
		flightcontrol.pitchControl(0.00, telemetry);
	}

	// Run the yaw controller to be fixed yaw
	if (yawStart != -999 && pwm_desired[0] >= 2900 && pwm_desired[0] <= 3000)
		flightcontrol.yawControl(yawStart, telemetry);
}

// Returns a 0 if the current action had no errors
// else returns a 1
uint8_t MissionControl::runMission() 
{
	// If altimeter isn't working, stop

	// Get the new UAVTalk data
	uavtalk.read(telemetry);

	if (yawStart == -999) yawStart = telemetry.uav_heading;

	/*uint32_t temp; 
	memcpy(&temp, &telemetry.uav_alt, sizeof(float));
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

	/*uint32_t temp; 
	float a = telemetry.uav_alt - ground_reference
	memcpy(&temp, &a, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/

	/*uint32_t input = pwm_desired[1];
	//float input = telemetry.uav_alt;
	//float yaw = telemetry.uav_heading - yawStart;
	uint32_t temp; 
	memcpy(&temp, &input, sizeof(float));
	UART::writeByte(temp >> 24);
	UART::writeByte(temp >> 16);
	UART::writeByte(temp >> 8);
	UART::writeByte(temp);*/

	switch(mission.actions[mission.actionIndex].type) {
		case ACTION_TAKEOFF:
			// Run the safety checks
			// If problems occurred, return a 1
			//if (runSafetyChecks()) return 1;
			
			// Set the goal altitude and run the altitude controller
			flightcontrol.altitudeControl(0.50, telemetry);

			// Set the roll and pitch angles to be 0.00
			flightcontrol.rollControl(0.00, telemetry);
			flightcontrol.pitchControl(0.00, telemetry);

			// Set the yaw to be stationary
			if (yawStart != -999)
				flightcontrol.yawControl(yawStart, telemetry);

			break;
		case ACTION_HOVER:
			// Set the goal altitude as 

			break;
		case ACTION_WP:

			break;
		case ACTION_LAND:

			break;
		default:
			// Invalid type
			break;
	}

	return 0;
}

// Returns 0 if no failures were found else returns 1
// TODO: Write any errors to a log file or something
uint8_t MissionControl::runSafetyChecks() 
{
	// Check if the battery is over 15%
	if (telemetry.uav_bat < 15) return 1;
	// Check if number of satellites is at least 7
	if (telemetry.uav_satellites_visible < 7) return 1;

	// Communication check is done in the main.cpp

	return 0;
}

void MissionControl::setAltitude(const float alt)
{
	telemetry.uav_alt = alt;
}

void MissionControl::setAltOffset(const float off)
{
	//float tempalt = altimeter.getAltitude();
	altimeterOffset = off;
}
