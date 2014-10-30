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

typedef enum 
{
	ACTION_TAKEOFF,
	ACTION_LAND,
	ACTION_HOVER,
	ACTION_WP
} MissionActionType;

typedef struct _MissionAction
{
	MissionActionType type;
	uint16_t altitude;
	int16_t waypointX;
	int16_t waypointY;
} MissionAction;

// Struct holding the mission's actions
typedef struct _Mission
{
	// TODO: Add name of the mission
	uint8_t numofActions;
	uint8_t actionIndex;
	MissionAction actions[];
} Mission;

class MissionControl 
{
	public:
		// Constructor to init mission
		MissionControl();
		
		// Inteprets the current action and calls the relevant controller
		uint8_t runMission();

	private:
		// Runs safety checks before beginning a mission
		uint8_t runSafetyChecks();

		Mission mission;
		UAVTalk uavtalk;
		MPL3115A2 altimeter;
		TelemetryData telemetry;

};

#endif
