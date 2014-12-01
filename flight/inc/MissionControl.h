/*
 *	File: MissionControl.h
 *	MDP Pillar Technology 2014
 *	Brief: Holds the mission actions and intepreter for the actions
*/

#ifndef _MISSIONCONTROL_H
#define _MISSIONCONTROL_H

#include <stdint.h>
#include "inc/UAVTalk.h"
#include "inc/MPL3115A2.h"
#include "inc/FlightControl.h"

typedef enum 
{
	ACTION_INIT,
	ACTION_TAKEOFF,
	ACTION_LAND,
	ACTION_HOVER,
	ACTION_WP,
	ACTION_END
} MissionActionType;

// Struct for an action
typedef struct _MissionAction
{
	MissionActionType type;	// Type of action (landing, hover, etc)
	float altitude;		// Altitude action takes place
	double waypointLong;	// Longitude of position
	double waypointLat;		// Latitude of position
	float time;				// Variable time (specific to action type)
} MissionAction;

// Struct holding the mission's actions
typedef struct _Mission
{
	// TODO: Add name of the mission
	uint8_t numofActions;
	uint8_t actionIndex;
	MissionAction actions[5];	// TODO: Make this dynamic
	MissionAction action;
} Mission;

class MissionControl 
{
	public:
		// Constructor to init mission
		MissionControl();

		// Init function
		void init();
		
		// Inteprets the current action and calls the relevant controller
		uint8_t runMission();

		// Handles basic manual flight control
		void runManual();

		// Update the altitude from the ultrasound
		void setAltitude(const float alt);

		// Set the offset from the ultrasound
		void setAltOffset(const float off);

		void updateTelemetry();

	private:
		void controlWaypoint(const MissionAction& action);

		void moveLat(const MissionAction& action);
		void moveLng(const MissionAction& action);

		// Runs safety checks before beginning a mission
		uint8_t runSafetyChecks();

		Mission mission;
		FlightControl flightcontrol;

		UAVTalk uavtalk;
		MPL3115A2 altimeter;

		TelemetryData telemetry;
		uavtalk_message_t msg;

		bool altimeterError;
		float ultrasoundAlt;
		float altimeterOffset;

		float yawStart;

		int16_t hover_start;
		float hover_time;

		float landing_dest;

		const static float long_Kp = 1.2;
		const static float long_Kd = 0;
		const static float lat_Kp = 1;
		//double errorLong;
		//double errorLat;

		float startLon, startLat;
		float latdist, lngdist;
		float errorLat, errorLng;
		float lastLat, lastLng;
		uint32_t wpCounter;
};

#endif
