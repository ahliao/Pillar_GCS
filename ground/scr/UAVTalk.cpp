/*
 * File:		UAVTalk.cpp
 * Author:		Alex Liao
 * Desc:		Implements UAVTalk telemetry communication with OpenPilot
 *				and the Pillar GCS
 * Etc:			Code based on code written by Joerg-D. Rothfuchs
 */

#include <iostream>
#include <cmath>
#include <QtEndian>
#include "inc/UAVTalk.h"
#include "inc/mainwindow.h"
#include <iomanip>

using namespace std;

// CRC lookup table
const static uint8_t crc_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

UAVTalk::UAVTalk(MainWindow* parent=0) {
	last_gcstelemetrystats_send = 0;
	last_flighttelemetry_connect = 0;
	gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;
	gcstelemetrystats_objid = 0;
	gcstelemetrystats_obj_len = 0;
	gcstelemetrystats_obj_status = 0;
	flighttelemetrystats_obj_status = 0;

	uav_rssi = 0;
	uav_linkquality = 0;
	uav_failsafe = 0;
	uav_arm = 0;
	uav_flightmode = 0;
	uav_roll = 0;
	uav_pitch = 0;
	uav_heading = 0;
	uav_lat = 0;
	uav_lon = 0;
	uav_satellites_visible = 0;
	uav_fix_type = 0;
	uav_gpsheading = 0;
	uav_alt = 0;
	uav_groundspeed = 0;
	uav_bat = 0;
	uav_current = 0;
	uav_amp = 0;

	mainwindow = parent;

	// Make serialport and set the configs
	serial = new QSerialPort(this);

	of.open("logs/pitchdata.txt", std::ofstream::out);
	logindex = 0;
	time.start();
	//openSerialPort();

	connect(serial, SIGNAL(readyRead()), this, SLOT(read()));
}

UAVTalk::~UAVTalk() {
	closeSerialPort();
	of.close();
}

// Read from the serial stream
int UAVTalk::read() {
	uavtalk_message_t msg;
	static uint8_t crlf_count = 0;
	uint8_t show_prio_info = 0;
	bool telemetry_ok = false;

	// Grab data
	/*uint8_t c = readByte();
	cerr <<  c << endl;
	c = readByte();
	cerr <<  (int) c << endl;
	c = readByte();
	cerr <<  (int) c << endl;*/
	uint32_t temp = 0;
	float f;
	uint8_t c = readByte();
	temp |= c << 24;
	c = readByte();
	temp |= c << 16;
	c = readByte();
	temp |= c << 8;
	c = readByte();
	temp |= c;
	memcpy(&f, &temp, sizeof(float));
	//cerr << std::setprecision(11) << f << endl;
	cerr << f << endl;
	serial->clear(QSerialPort::Input);
	//serial->waitForReadyRead(50);
	/*double difference = 0;	// Time diff
	uint32_t temp = 0;
	float f = 0;
	uint8_t c = readByte();
	temp |= c << 24;
	c = readByte();
	temp |= c << 16;
	c = readByte();
	temp |= c << 8;
	c = readByte();
	temp |= c;
	memcpy(&f, &temp, sizeof(float));
	//c = readByte();
	cerr << f << endl;
	//difference = time.elapsed() / 1000.0;
	//of  << difference << ", " << f << endl;
	serial->clear(QSerialPort::Input);*/
	//mainwindow->updateAttitudeState(f, 0, 0);
	/*double difference = 0;	// Time diff
	serial->waitForReadyRead(50);
	while (serial->bytesAvailable() > 0) {
		// read in one byte
		uint8_t c = (uint8_t) serial->read(1).at(0);
		//uint8_t c = readByte();
		//cerr << (int) c << endl;

		// parse data to msg
		if (parse_char(c, &msg)) {
			//telemetry_ok = true;
			//lastpacketreceived = millis();
			// consume msg
			//cerr << msg.ObjID << endl;
			switch (msg.ObjID) {
				case FLIGHTTELEMETRYSTATS_OBJID:
				case FLIGHTTELEMETRYSTATS_OBJID_001:
					set_telemetrystats_values(msg.ObjID);
					switch (msg.Data[flighttelemetrystats_obj_status]) {
						case TELEMETRYSTATS_STATE_DISCONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_HANDSHAKEREQ;
							//uavtalk_send_gcstelemetrystats();
							break;
						case TELEMETRYSTATS_STATE_HANDSHAKEACK:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							//uavtalk_send_gcstelemetrystats();
							break;
						case TELEMETRYSTATS_STATE_CONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							//last_flighttelemetry_connect = millis();
							break;
					}
					break;
				case ATTITUDEACTUAL_OBJID:
				case ATTITUDESTATE_OBJID:
					//last_flighttelemetry_connect = millis();
					show_prio_info = 1;
					uav_roll =  get_float(&msg, ATTITUDEACTUAL_OBJ_ROLL);
					uav_pitch = get_float(&msg, ATTITUDEACTUAL_OBJ_PITCH);

					// log the pitch data
					difference = time.elapsed() / 1000.0;
					of  << difference << ", " << uav_pitch << endl;
					uav_heading	= get_float(&msg, ATTITUDEACTUAL_OBJ_YAW);
					mainwindow->updateAttitudeState(uav_roll, uav_pitch, uav_heading);
					break;
				case ACCELSTATE_OBJID:
					uav_accel_x = get_float(&msg, ACCELSTATE_OBJ_X);
					uav_accel_y = get_float(&msg, ACCELSTATE_OBJ_Y);
					uav_accel_z = get_float(&msg, ACCELSTATE_OBJ_Z);
					mainwindow->updateAccelState(uav_accel_x, uav_accel_y, uav_accel_z);
					break;
				case GYROSTATE_OBJID:
					uav_gyro_x = get_float(&msg, GYROSTATE_OBJ_X);
					uav_gyro_y = get_float(&msg, GYROSTATE_OBJ_Y);
					uav_gyro_z = get_float(&msg, GYROSTATE_OBJ_Z);
					break;
				case FLIGHTSTATUS_OBJID:
				case FLIGHTSTATUS_OBJID_001:
				case FLIGHTSTATUS_OBJID_002:
				case FLIGHTSTATUS_OBJID_003:
				case FLIGHTSTATUS_OBJID_004:
				case FLIGHTSTATUS_OBJID_005:
					uav_arm = get_int8(&msg, FLIGHTSTATUS_OBJ_ARMED);
					//remap flight modes id to Ghettostation ones
					uav_flightmode = get_int8(&msg, FLIGHTSTATUS_OBJ_FLIGHTMODE);
					switch (get_int8(&msg, FLIGHTSTATUS_OBJ_FLIGHTMODE)) {
						case 0: uav_flightmode = 0;  break;   //manual
						case 1: uav_flightmode = 5;  break;   //stabilized 1
						case 2: uav_flightmode = 6;  break;   //stabilized 2
						case 3: uav_flightmode = 7;  break;   //stabilized 3
						case 4: uav_flightmode = 16; break;   //autotune (unknown)
						case 5: uav_flightmode = 8;  break;   //altitude hold
						case 6: uav_flightmode = 16; break;   //velocity control (unknown)
						case 7: uav_flightmode = 9;  break;   //pos hold
						case 8: uav_flightmode = 13; break;   //RTH
						case 9: uav_flightmode = 10; break;   //pathplanner (auto)
					}
					//if (msg.ObjID==FLIGHTSTATUS_OBJID_004) {
					uav_failsafe = (get_int8(&msg, FLIGHTSTATUS_OBJ_CONTROLSOURCE) == 1) ? 1 : 0; // Taulabs only
					mainwindow->updateFlightStatus(uav_arm, uav_flightmode);
					//}
					break;
				case MANUALCONTROLCOMMAND_OBJID: // OP
					// uav_chan5_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_4);
					// uav_chan6_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_6);
					// uav_chan7_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_7);
					// uav_chan8_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_8);
					break;
				case MANUALCONTROLCOMMAND_OBJID_001: //Taulabs
					uav_rssi = (uint8_t) get_int16( &msg, MANUALCONTROLCOMMAND_OBJ_001_RSSI);
					if ( uav_rssi > 100 ) uav_rssi = 0; // rssi > 100 means it entered taulabs failsafe mode
					// uav_chan5_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_4);
					// uav_chan6_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_6);
					// uav_chan7_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_7);
					// uav_chan8_raw = uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_001_CHANNEL_8);

					break;
				case GPSPOSITION_OBJID:
				case GPSPOSITION_OBJID_001:
				case GPSPOSITIONSENSOR_OBJID:
					// Put lat and lon into array to get the median
					lat_array[gps_index++] = (double)(get_int32(&msg, GPSPOSITION_OBJ_LAT)/1.0e7);
					lon_array[gps_index++] = (double)(get_int32(&msg, GPSPOSITION_OBJ_LON)/1.0e7);
					if (gps_index > 4) gps_index = 0;

					// Find the median of the array
					// Sort array 
					for (int i = 0; i < 5; ++i) sorted_array[i] = lat_array[i];
					for (int i = 0; i < 5; ++i) {
						for (int j = i + 1; j < 5; ++j) {
							if (sorted_array[i] > sorted_array[j]) {
								float temp = sorted_array[j];
								sorted_array[j] = sorted_array[i];
								sorted_array[i] = temp;
							}
						}
					}
					uav_lat = sorted_array[2];

					// Sort array 
					for (int i = 0; i < 5; ++i) sorted_array[i] = lat_array[i];
					for (int i = 0; i < 5; ++i) {
						for (int j = i + 1; j < 5; ++j) {
							if (sorted_array[i] > sorted_array[j]) {
								float temp = sorted_array[j];
								sorted_array[j] = sorted_array[i];
								sorted_array[i] = temp;
							}
						}
					}
					uav_lon = sorted_array[2];

					//uav_lat	= (double) (get_int32(&msg, GPSPOSITION_OBJ_LAT) / 1.0e7);
					//uav_lon = (double) (get_int32(&msg, GPSPOSITION_OBJ_LON) / 1.0e7);
					uav_satellites_visible	= (int8_t) get_int8(&msg, GPSPOSITION_OBJ_SATELLITES);
					uav_fix_type = (uint8_t) get_int8(&msg, GPSPOSITION_OBJ_STATUS);
					uav_gpsheading = (double) get_int32(&msg, GPSPOSITION_OBJ_HEADING);
#ifndef BARO_ALT
					uav_alt	= get_float(&msg, GPSPOSITION_OBJ_ALTITUDE);
#endif
					uav_groundspeed	= (double)get_int32(&msg, GPSPOSITION_OBJ_GROUNDSPEED);
					mainwindow->updateGPSState(uav_lat, uav_lon, uav_satellites_visible,
							uav_gpsheading, uav_alt, uav_groundspeed, uav_fix_type);
					break;

				case FLIGHTBATTERYSTATE_OBJID:
					uav_bat	= (int16_t) (1000.0 * get_float(&msg, FLIGHTBATTERYSTATE_OBJ_VOLTAGE));
					uav_current	= (int16_t) (100.0 * get_float(&msg, FLIGHTBATTERYSTATE_OBJ_CURRENT));
					uav_amp	= (int16_t) get_float(&msg, FLIGHTBATTERYSTATE_OBJ_CONSUMED_ENERGY);
					mainwindow->updateBatteryState(uav_bat, uav_current, uav_amp);
					break;

				case BAROALTITUDE_OBJID:
				case BAROSENSOR_OBJID:
#ifdef BARO_ALT
					uav_alt	= (int32_t) round (get_float(&msg, BAROALTITUDE_OBJ_ALTITUDE) * 100.0f);
#endif
					break;
				case OPLINKSTATUS_OBJID:
				case OPLINKSTATUS_OBJID_001:
					uav_rssi = get_int8(&msg, OPLINKSTATUS_OBJ_RSSI);
					uav_linkquality	= (uint8_t) get_int8(&msg, OPLINKSTATUS_OBJ_LINKQUALITY);
					uav_linkstate = (uint8_t) get_int8(&msg, OPLINKSTATUS_OBJ_LINKSTATE);
					mainwindow->updateOPLinkStatus(uav_rssi, uav_linkquality, uav_linkstate);
					break;

			}
			if (msg.MsgType == UAVTALK_TYPE_OBJ_ACK) {
				respond_object(&msg, UAVTALK_TYPE_ACK);
			}
			// Return a 1 if message updated any data

			return 1;
		}

		//delayMicroseconds(190);  // wait at least 1 byte
	}*/

	// check connect timeout
	/*if (last_flighttelemetry_connect + FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT < millis()) {
		gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;
		show_prio_info = 1;
	}

	// periodically send gcstelemetrystats
	if (last_gcstelemetrystats_send + GCSTELEMETRYSTATS_SEND_PERIOD < millis()) {
		uavtalk_send_gcstelemetrystats();
	}*/

	//return show_prio_info;
	//return msg;
	return 0;
}

uint8_t UAVTalk::readByte(void) {
	serial->waitForReadyRead(1);
	QByteArray data = serial->read(1);
	return data.at(0);
}

void UAVTalk::write(uint8_t in) {
	char c = in;
	serial->write(&c);
}

// Get the state
int UAVTalk::state(void) {
	return 0;
}

void UAVTalk::openSerialPort() {
	serial->setPortName("/dev/ttyUSB0");
	serial->setBaudRate(QSerialPort::Baud38400);
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);

	// open the serial port
	if (serial->open(QIODevice::ReadWrite)) {
		cout << "Successfully opened serial port\n";
		cout << serial->portName().toStdString() << endl;
	} else {
		cout << "Error opening serial port\n";
		cout << serial->errorString().toStdString() << endl;
	}

}

void UAVTalk::closeSerialPort() {
	cout << "Closing serial port\n";
	serial->close();
}

int8_t UAVTalk::get_int8(uavtalk_message_t *msg, int pos) {
	return msg->Data[pos+2];
}

// return an int16 from the ms
int16_t UAVTalk::get_int16(uavtalk_message_t *msg, int pos) {
	int16_t i;
	memcpy(&i, msg->Data+pos+2, sizeof(int16_t));
	return i;
}

int32_t UAVTalk::get_int32(uavtalk_message_t *msg, int pos) {
	int32_t i;
	memcpy(&i, msg->Data+pos+2, sizeof(int32_t));
	return i;
}

// return an float from the ms
float UAVTalk::get_float(uavtalk_message_t *msg, int pos) {
	float f;
	// What the memcpy is doing
	//uint32_t temp;
	//temp = *(msg->Data+pos+2) | (*(msg->Data+pos+1+2) << 8) |
	//	(*(msg->Data+pos+2+2) << 16) | (*(msg->Data+pos+3+2) << 24);
	memcpy(&f, msg->Data+pos+2, sizeof(float));
	return f;
}

// send the uavtalk message msg
void UAVTalk::send_msg(uavtalk_message_t *msg) {
	uint8_t *d;
	uint8_t i;
	char c;

	c = (char) (msg->Sync);
	serial->write(&c);
	msg->Crc = crc_table[0];
	c = (char) (msg->MsgType);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) (msg->Length & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) ((msg->Length >> 8) & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) (msg->ObjID & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) ((msg->ObjID >> 8) & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) ((msg->ObjID >> 16) & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (char) ((msg->ObjID >> 24) & 0xff);
	serial->write(&c);
	msg->Crc = crc_table[msg->Crc ^ c];
	if (msg->Length > 8) {
		d = msg->Data;
		for (i=0; i<msg->Length-8; i++) {
			c = *d++;
			serial->write(&c);
			msg->Crc = crc_table[msg->Crc ^ c];
		}
	}
	c = msg->Crc;
	serial->write(&c);
}

void UAVTalk::respond_object(uavtalk_message_t *msg_to_respond, uint8_t type) {
	uavtalk_message_t msg;

	msg.Sync 	= UAVTALK_SYNC_VAL;
	msg.MsgType = type;
	msg.Length 	= RESPOND_OBJ_LEN;
	msg.ObjID	= msg_to_respond->ObjID;

	send_msg(&msg);
}

void UAVTalk::send_gcstelemetrystats(void) {

}

void UAVTalk::set_telemetrystats_values(uint32_t ObjID) {

}

uint8_t UAVTalk::parse_char(uint8_t c, uavtalk_message_t *msg) {
	static uint8_t status = UAVTALK_PARSE_STATE_WAIT_SYNC;
	static uint8_t crc = 0;
	static uint8_t cnt = 0;

	switch(status) {
		case UAVTALK_PARSE_STATE_WAIT_SYNC:
			if (c == UAVTALK_SYNC_VAL) {
				status = UAVTALK_PARSE_STATE_GOT_SYNC;
				msg->Sync = c;
				crc = crc_table[0 ^ c];
			}
			break;
		case UAVTALK_PARSE_STATE_GOT_SYNC:
			crc = crc_table[crc ^ c];
			if ((c & UAVTALK_TYPE_MASK) == UAVTALK_TYPE_VER) {
				status = UAVTALK_PARSE_STATE_GOT_MSG_TYPE;
				msg->MsgType = c;
				cnt = 0;
			} else {
				status = UAVTALK_PARSE_STATE_WAIT_SYNC;
			}
			break;
		case UAVTALK_PARSE_STATE_GOT_MSG_TYPE:
			crc = crc_table[crc ^ c];
			cnt++;
			if (cnt < 2) {
				msg->Length = ((uint16_t) c);
			} else {
				msg->Length += ((uint16_t) c) << 8;
				if ((msg->Length < 8) || (msg->Length > 255 + 8)) {
					// Drop corrupted messages:
					// Minimal length is 8 (headers)
					// Maximum is 8 (headers) + 255 (Data) + 2 (Optional Instance Id)
					// As we are not parsing Instance Id, 255 is a hard maximum. 
					status = UAVTALK_PARSE_STATE_WAIT_SYNC;
				} else {
					status = UAVTALK_PARSE_STATE_GOT_LENGTH;
					cnt = 0;
				}
			}
			break;
		case UAVTALK_PARSE_STATE_GOT_LENGTH:
			crc = crc_table[crc ^ c];
			cnt++;
			switch (cnt) {
				case 1:
					msg->ObjID = ((uint32_t) c);
					break;
				case 2:
					msg->ObjID += ((uint32_t) c) << 8;
					break;
				case 3:
					msg->ObjID += ((uint32_t) c) << 16;
					break;
				case 4:
					msg->ObjID += ((uint32_t) c) << 24;
					if (msg->Length == 8) { // no data exists
						status = UAVTALK_PARSE_STATE_GOT_DATA;
					} else {
						status = UAVTALK_PARSE_STATE_GOT_OBJID;
					}
					cnt = 0;
					break;
			}
			break;
		case UAVTALK_PARSE_STATE_GOT_OBJID:
		case UAVTALK_PARSE_STATE_GOT_INSTID:
			crc = crc_table[crc ^ c];
			cnt++;
			msg->Data[cnt - 1] = c;
			if (cnt >= msg->Length - 8) {
				status = UAVTALK_PARSE_STATE_GOT_DATA;
				cnt = 0;
			}
			break;
		case UAVTALK_PARSE_STATE_GOT_DATA:
			msg->Crc = c;
			status = UAVTALK_PARSE_STATE_GOT_CRC;
			break;
	}

	if (status == UAVTALK_PARSE_STATE_GOT_CRC) {
		status = UAVTALK_PARSE_STATE_WAIT_SYNC;
		//cerr << (int) crc << "   " << (int) msg->Crc << endl;
		if (crc == msg->Crc) {
			return msg->Length;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}


