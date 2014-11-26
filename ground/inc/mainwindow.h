/*
 * File:		mainwindow.h
 * Author:		Alex Liao
 * Desc:		Main GUI window for the Pillar GCS
 * Etc:			Requires Qt library
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QtGlobal>
#include <QMainWindow>
#include <QDialog>
#include <QLabel>

#include "inc/UAVTalk.h"

//namespace Ui {
//	class MainWindow;
//}

class MainWindow : public QDialog
{
	Q_OBJECT

	public:
		MainWindow(QWidget *parent = 0);
		~MainWindow();

		void updateAttitudeState(const float r, 
				const float pitch, const float yaw);

		void updateAccelState(const float x,
				const float y, const float z);

		void updateGPSState(const double lat, const double lon,
				const int satellites, const int gpsheading,
				const int alt, const int speed, const int fix);

		void updateFlightStatus(const int armed, const int mode);

		void updateOPLinkStatus(const int rssi, const int quality,
				const int state);

		void updateBatteryState(const int bat, const int current,
				const int amp);

	private:
		void initActionsConnections();

	private:
		QLabel* rollLabel;
		QLabel* pitchLabel;
		QLabel* yawLabel;

		QLabel* accelXLabel;
		QLabel* accelYLabel;
		QLabel* accelZLabel;

		QLabel* gyroXLabel;
		QLabel* gyroYLabel;
		QLabel* gyroZLabel;

		QLabel* latLabel;
		QLabel* lonLabel;
		QLabel* satellitesLabel;
		QLabel* gpsheadingLabel;
		QLabel* fixLabel;
		QLabel* altLabel;
		QLabel* groundspeedLabel;

		QLabel* rssiLabel;
		QLabel* linkqualityLabel;
		QLabel* linkstateLabel;

		QLabel* armLabel;
		QLabel* flightmodeLabel;

		QLabel* batLabel;
		QLabel* currentLabel;
		QLabel* ampLabel;

		//Ui::MainWindow *ui;

		UAVTalk* uavtalk;
};

#endif
