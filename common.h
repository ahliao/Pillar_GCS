#ifndef _COMMON_H
#define _COMMON_H

//#include <stdint.h>

// These are made public for debugging
extern int8_t uav_rssi;
extern uint8_t uav_linkquality;
extern uint8_t uav_linkstate;

extern uint8_t uav_failsafe;
extern uint8_t uav_arm;
extern uint8_t uav_flightmode;

extern float uav_roll;
extern float uav_pitch;
extern float uav_heading;

// Accelerometer 
extern float uav_accel_x;
extern float uav_accel_y;
extern float uav_accel_z;

// Gyroscope
extern float uav_gyro_x;
extern float uav_gyro_y;
extern float uav_gyro_z;

// GPS data
extern int32_t uav_lat;
extern int32_t uav_lon;
extern uint8_t uav_satellites_visible;
extern uint8_t uav_fix_type;
extern int16_t uav_gpsheading;
extern int32_t uav_alt;
extern uint16_t uav_groundspeed;

// Battery and power data
extern uint16_t uav_bat;
extern uint16_t uav_current;
extern uint16_t uav_amp;

#endif
