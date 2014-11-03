#ifndef _TELEMETRYDATA
#define _TELEMETRYDATA

#include <stdint.h>

typedef struct _TelemeteryData
{
	int8_t uav_rssi;
	uint8_t uav_linkquality;
	uint8_t uav_linkstate;

	uint8_t uav_failsafe;
	uint8_t uav_arm;
	uint8_t uav_flightmode;

	// GPS data
	uint8_t uav_satellites_visible;
	uint8_t uav_fix_type;
	int16_t uav_gpsheading;
	uint16_t uav_groundspeed;

	// Battery and power data
	uint16_t uav_bat;
	uint16_t uav_current;
	uint16_t uav_amp;

	int32_t uav_lat;
	int32_t uav_lon;
	float uav_alt;

	float uav_roll;
	float uav_pitch;
	float uav_heading;

	// Accelerometer 
	float uav_accel_x;
	float uav_accel_y;
	float uav_accel_z;

	// Gyroscope
	float uav_gyro_x;
	float uav_gyro_y;
	float uav_gyro_z;
} TelemetryData;

#endif
