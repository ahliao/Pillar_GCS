#include "inc/MissionControl.h"
//#include "inc/common.h"

MissionControl::MissionControl()
{
	// TODO: load in mission from EEPROM or PI
	mission.numofActions = 3;
	mission.actionIndex = 0;
	mission.actions[0] = ACTION_TAKEOFF;
	mission.actions[1] = ACTION_HOVER;
	mission.actions[2] = ACTION_LAND;
}

// Returns a 0 if the current action had no errors
// else returns a 1
uint8_t MissionControl::runMission() 
{
	// Get the new UAVTalk data
	uavtalk.read();

	return 0;
}

// Returns 0 if no failures were found else returns 1
// TODO: Write any errors to a log file or something
uint8_t MissionControl::runSafetyChecks() 
{
	// Check if the battery is over 15%
	if (uavtalk.uav_bat < 15) return 1;
	// Check if number of satellites is at least 7
	if (uavtalk.uav_satellites_visible < 7) return 1;
	// Check if communication to manual controller is active
	// TODO: Check this in the main function?
	//if (autopilot_state != AUTOPILOT_EMERGENCY)

	return 0;
}
