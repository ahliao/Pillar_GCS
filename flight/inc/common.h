#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

// These are made public for debugging
int8_t uav_rssi;
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
uint16_t uav_amp;

#endif
