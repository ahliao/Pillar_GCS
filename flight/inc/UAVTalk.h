/*
 * File:		UAVTalk.hpp
 * Author:		Alex Liao
 * Desc:		Implements UAVTalk telemetry communication with OpenPilot
 *				and the onboard autonomous flight controller
 * Etc:			Code based on code written by Joerg-D. Rothfuchs
 */

#ifndef UAVTALK_H
#define UAVTALK_H

#include <stdint.h>
#include "inc/TelemetryData.h"

#define UAVTALK_SYNC_VAL				0x3C
#define UAVTALK_TYPE_MASK				0xF8
#define UAVTALK_TYPE_VER				0x20

#define UAVTALK_TYPE_OBJ				(UAVTALK_TYPE_VER | 0x00)
#define UAVTALK_TYPE_OBJ_REQ			(UAVTALK_TYPE_VER | 0x01)
#define UAVTALK_TYPE_OBJ_ACK			(UAVTALK_TYPE_VER | 0x02)
#define UAVTALK_TYPE_ACK				(UAVTALK_TYPE_VER | 0x03)
#define UAVTALK_TYPE_NACK				(UAVTALK_TYPE_VER | 0x04)

#define	FLIGHTTELEMETRYSTATS_OBJID          0x2F7E2902
#define FLIGHTTELEMETRYSTATS_OBJID_001      0x6737BB5A
#define	GCSTELEMETRYSTATS_OBJID             0xABC72744
#define GCSTELEMETRYSTATS_OBJID_001         0xCAD1DC0A      // different ID for 14.02.01
#define	ATTITUDEACTUAL_OBJID                0x33DAD5E6
#define ATTITUDESTATE_OBJID                 0xD7E0D964      // new name since OP VERSION_RELEASE_14_02_1
#define	FLIGHTSTATUS_OBJID                  0x9B6A127E      // Op
#define	FLIGHTSTATUS_OBJID_001              0x0ED79A04      // Op next
#define	FLIGHTSTATUS_OBJID_002              0x1B7AEB74      // OP next
#define FLIGHTSTATUS_OBJID_003              0x0B37AA16      // OP
#define FLIGHTSTATUS_OBJID_004              0x6CECFBC2      // Taulabs 
#define FLIGHTSTATUS_OBJID_005              0xC5FF2D54      // Taulabs current next.
#define	MANUALCONTROLCOMMAND_OBJID          0x5C2F58AC      // Taulabs ( only used for RSSI )
#define MANUALCONTROLCOMMAND_OBJID_001      0xB8C7F78A      // different ID for 14.02.01
#define GPSPOSITION_OBJID                   0xE2A323B6
#define GPSPOSITION_OBJID_001               0x40BCC84E      // new Taulabs next ID.
#define GPSPOSITIONSENSOR_OBJID             0x1A5748CE      // new name since VERSION_RELEASE_14_02_1
#define AIRSPEEDACTUAL_OBJID                0x133A3280
#define AIRSPEEDACTUAL_OBJID_001			0xC7009F28
#define FLIGHTBATTERYSTATE_OBJID            0xD2083596
#define FLIGHTBATTERYSTATE_OBJID_001		0x26962352
#define BAROALTITUDE_OBJID                  0x99622E6A
#define BAROSENSOR_OBJID                    0x48120EA6      // new name since VERSION_RELEASE_14_02_1

#define OPLINKSTATUS_OBJID                  0x669C55E2
#define OPLINKSTATUS_OBJID_001              0xBE2584BA

#define ACCELSTATE_OBJID					0xAD3C0E06

#define GYROSTATE_OBJID						0x8C2D810A

#define FLIGHTTELEMETRYSTATS_OBJ_LEN            21
#define	FLIGHTTELEMETRYSTATS_OBJ_STATUS			20
#define FLIGHTTELEMETRYSTATS_OBJ_LEN_001        37          // different since VERSION_RELEASE_14_02_1
#define FLIGHTTELEMETRYSTATS_OBJ_STATUS_001     36          // different since VERSION_RELEASE_14_02_1

#define BAROALTITUDE_OBJ_ALTITUDE			0
#define BAROALTITUDE_OBJ_TEMPERATURE		4
#define BAROALTITUDE_OBJ_PRESSURE			8

#define FLIGHTBATTERYSTATE_OBJ_VOLTAGE			0
#define FLIGHTBATTERYSTATE_OBJ_CURRENT			4
#define FLIGHTBATTERYSTATE_OBJ_BOARD_SUPPLY_VOLTAGE	8
#define FLIGHTBATTERYSTATE_OBJ_PEAK_CURRENT		12
#define FLIGHTBATTERYSTATE_OBJ_AVG_CURRENT		16
#define FLIGHTBATTERYSTATE_OBJ_CONSUMED_ENERGY		20
#define FLIGHTBATTERYSTATE_OBJ_ESTIMATED_FLIGHT_TIME	24

#define GCSTELEMETRYSTATS_OBJ_LEN                       21
#define GCSTELEMETRYSTATS_OBJ_STATUS                    20
#define GCSTELEMETRYSTATS_OBJ_LEN_001                   37              // different since VERSION_RELEASE_14_02_1
#define GCSTELEMETRYSTATS_OBJ_STATUS_001                36              // different since VERSION_RELEASE_14_02_1

#define	ATTITUDEACTUAL_OBJ_ROLL				16
#define	ATTITUDEACTUAL_OBJ_PITCH			20
#define	ATTITUDEACTUAL_OBJ_YAW				24

#define	FLIGHTSTATUS_OBJ_ARMED              0
#define	FLIGHTSTATUS_OBJ_FLIGHTMODE         1
#define FLIGHTSTATUS_OBJ_CONTROLSOURCE      2               //Taulabs only

#define	GPSPOSITION_OBJ_LAT					0
#define	GPSPOSITION_OBJ_LON					4
#define	GPSPOSITION_OBJ_ALTITUDE			8
#define	GPSPOSITION_OBJ_GEOIDSEPARATION		12
#define	GPSPOSITION_OBJ_HEADING				16
#define	GPSPOSITION_OBJ_GROUNDSPEED			20
#define	GPSPOSITION_OBJ_PDOP				24
#define	GPSPOSITION_OBJ_HDOP				28
#define	GPSPOSITION_OBJ_VDOP				32
#define	GPSPOSITION_OBJ_STATUS				36
#define	GPSPOSITION_OBJ_SATELLITES			37

#define	OPLINKSTATUS_OBJ_RSSI				99
#define	OPLINKSTATUS_OBJ_LINKQUALITY			100
#define OPLINKSTATUS_OBJ_LINKSTATE				101

#define MANUALCONTROLCOMMAND_OBJ_THROTTLE               0
#define MANUALCONTROLCOMMAND_OBJ_ROLL                   4
#define MANUALCONTROLCOMMAND_OBJ_PITCH                  8
#define MANUALCONTROLCOMMAND_OBJ_YAW                    12
#define MANUALCONTROLCOMMAND_OBJ_COLLECTIVE             16
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_0              20
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_1              22
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_2              24
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_3              26
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_4              28
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_5              30
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_6              32
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_7              34
#define MANUALCONTROLCOMMAND_OBJ_CHANNEL_8              36

#define MANUALCONTROLCOMMAND_OBJ_001_THROTTLE               0
#define MANUALCONTROLCOMMAND_OBJ_001_ROLL                   4
#define MANUALCONTROLCOMMAND_OBJ_001_PITCH                  8
#define MANUALCONTROLCOMMAND_OBJ_001_YAW                    12
#define MANUALCONTROLCOMMAND_OBJ_001_RAWRSSI                16
#define MANUALCONTROLCOMMAND_OBJ_001_COLLECTIVE             20
#define MANUALCONTROLCOMMAND_OBJ_001_RSSI                   24
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_0              26
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_1              28
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_2              30
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_3              32
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_4              34
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_5              36
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_6              38
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_7              40
#define MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_8              42

#define ACCELSTATE_OBJ_X				0
#define ACCELSTATE_OBJ_Y				4
#define ACCELSTATE_OBJ_Z				8

#define GYROSTATE_OBJ_X						0
#define GYROSTATE_OBJ_Y						4
#define GYROSTATE_OBJ_Z						8

#define	RESPOND_OBJ_LEN					8

// Structure for the UAVTalk message
typedef struct __uavtalk_message {
	uint8_t Sync;
	uint8_t MsgType;
	uint16_t Length;
	uint32_t ObjID;
	uint16_t InstID;
	uint8_t Data[255];
	uint8_t Crc;
} uavtalk_message_t;

// Enumeration options for field FlightMode 
// From build/uavobject-synthetics/flight/flightstatus.h
typedef enum { 
	FLIGHTSTATUS_FLIGHTMODE_MANUAL=0, 
	FLIGHTSTATUS_FLIGHTMODE_STABILIZED1=1, 
	FLIGHTSTATUS_FLIGHTMODE_STABILIZED2=2, 
	FLIGHTSTATUS_FLIGHTMODE_STABILIZED3=3, 
	FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE=4, 
	FLIGHTSTATUS_FLIGHTMODE_ALTITUDEHOLD=5, 
	FLIGHTSTATUS_FLIGHTMODE_VELOCITYCONTROL=6, 
	FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD=7,
	FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME=8,
	FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER=9
} FlightStatusFlightModeOptions;

// Enumeration options for field ControlSource 
// From build/uavobject-synthetics/flight/flightstatus.h
typedef enum { 
	FLIGHTSTATUS_CONTROLSOURCE_GEOFENCE=0, 
	FLIGHTSTATUS_CONTROLSOURCE_FAILSAFE=1, 
	FLIGHTSTATUS_CONTROLSOURCE_TRANSMITTER=2, 
	FLIGHTSTATUS_CONTROLSOURCE_TABLET=3 
} FlightStatusControlSourceOptions;

typedef enum {
	UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
	UAVTALK_PARSE_STATE_GOT_SYNC,
	UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
	UAVTALK_PARSE_STATE_GOT_LENGTH,
	UAVTALK_PARSE_STATE_GOT_OBJID,
	UAVTALK_PARSE_STATE_GOT_INSTID,
	UAVTALK_PARSE_STATE_GOT_DATA,
	UAVTALK_PARSE_STATE_GOT_CRC
} uavtalk_parse_state_t;

typedef enum {
	TELEMETRYSTATS_STATE_DISCONNECTED = 0,
	TELEMETRYSTATS_STATE_HANDSHAKEREQ,
	TELEMETRYSTATS_STATE_HANDSHAKEACK,
	TELEMETRYSTATS_STATE_CONNECTED
} telemetrystats_state_t;

class UAVTalk
{
	public:
		UAVTalk();

		~UAVTalk();

		// Read from the serial stream
		int read(TelemetryData& data);
		//int read(uavtalk_message_t& msg);

		// Get the state
		int state(void);

		//TelemetryData data;
		// These are made public for debugging
		/*int8_t uav_rssi;
		uint8_t uav_linkquality;
		uint8_t uav_linkstate;

		uint8_t uav_failsafe;
		uint8_t uav_arm;
		uint8_t uav_flightmode;

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

		// GPS data
		int32_t uav_lat;
		int32_t uav_lon;
		uint8_t uav_satellites_visible;
		uint8_t uav_fix_type;
		int16_t uav_gpsheading;
		int32_t uav_alt;
		uint16_t uav_groundspeed;

		// Battery and power data
		uint16_t uav_bat;
		uint16_t uav_current;
		uint16_t uav_amp;*/

	private:
		// return an int8 from the ms
		inline int8_t get_int8(uavtalk_message_t *msg, int pos);

		// return an int16 from the ms
		inline int16_t get_int16(uavtalk_message_t *msg, int pos);

		inline int32_t get_int32(uavtalk_message_t *msg, int pos);

		// return an float from the ms
		inline float get_float(uavtalk_message_t *msg, int pos);

		// send the uavtalk message msg
		void send_msg(uavtalk_message_t *msg);

		// Respond with an UAVObject
		void respond_object(uavtalk_message_t *msg_to_respond, uint8_t type);

		uint8_t parse_char(uint8_t c, uavtalk_message_t *msg);

		// TODO: Figure out if these are important
		unsigned long last_gcstelemetrystats_send;
		unsigned long last_flighttelemetry_connect;
		uint8_t gcstelemetrystatus;
		uint32_t gcstelemetrystats_objid;
		uint8_t gcstelemetrystats_obj_len;
		uint8_t gcstelemetrystats_obj_status;
		uint8_t flighttelemetrystats_obj_status;
};

#endif
