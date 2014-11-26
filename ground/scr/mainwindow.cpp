#include "inc/mainwindow.h"
#include <QVBoxLayout>
//#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QDialog(parent)
    
{
    /*ui->setupUi(this);

    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionQuit->setEnabled(true);
    ui->actionConfigure->setEnabled(true);*/

    //initActionsConnections();
	rollLabel = new QLabel("Roll: 0.00");
	pitchLabel = new QLabel("Pitch: 0.00");
	yawLabel = new QLabel("Yaw: 0.00");
	accelXLabel = new QLabel("Accel X: 0.00");
	accelYLabel = new QLabel("Accel Y: 0.00");
	accelZLabel = new QLabel("Accel Z: 0.00");
	armLabel = new QLabel("Armed: 0");
	flightmodeLabel = new QLabel("Flight Mode: 0");
	batLabel = new QLabel("Battery: 0");
	currentLabel = new QLabel("Current: 0");	// Don't know the difference
	ampLabel = new QLabel("Amps: 0");
	latLabel = new QLabel("Lat: 0");
	lonLabel = new QLabel("Lon: 0");
	satellitesLabel = new QLabel("Satellites: 0");
	gpsheadingLabel = new QLabel("GPS Heading: 0.00");
	fixLabel = new QLabel("Fix: 0");
	altLabel = new QLabel("Altitude: 0.00");
	groundspeedLabel = new QLabel("Ground Speed: 0.00");
	rssiLabel = new QLabel("RSSI: 0");
	linkqualityLabel = new QLabel("Link Quality: 0");
	linkstateLabel = new QLabel("Link State: 0");
	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addWidget(rollLabel);
	mainLayout->addWidget(pitchLabel);
	mainLayout->addWidget(yawLabel);
	mainLayout->addWidget(accelXLabel);
	mainLayout->addWidget(accelYLabel);
	mainLayout->addWidget(accelZLabel);
	mainLayout->addWidget(armLabel);
	mainLayout->addWidget(flightmodeLabel);
	mainLayout->addWidget(batLabel);
	mainLayout->addWidget(currentLabel);
	mainLayout->addWidget(ampLabel);
	mainLayout->addWidget(latLabel);
	mainLayout->addWidget(lonLabel);
	mainLayout->addWidget(satellitesLabel);
	mainLayout->addWidget(gpsheadingLabel);
	mainLayout->addWidget(fixLabel);
	mainLayout->addWidget(altLabel);
	mainLayout->addWidget(groundspeedLabel);
	mainLayout->addWidget(rssiLabel);
	mainLayout->addWidget(linkqualityLabel);
	mainLayout->addWidget(linkstateLabel);
	setLayout(mainLayout);
	//setCentralWidget(rollLabel);
	resize(300,300);

	uavtalk = new UAVTalk(this);
	uavtalk->openSerialPort();
}

MainWindow::~MainWindow()
{
    //delete ui;
	delete uavtalk;
}

void MainWindow::updateAttitudeState(const float roll, 
		const float pitch, const float yaw) 
{
	rollLabel->setText(QString("Roll: %1").arg(roll));
	pitchLabel->setText(QString("Pitch: %1").arg(pitch));
	yawLabel->setText(QString("Yaw: %1").arg(yaw));
}

void MainWindow::updateAccelState(const float x, 
		const float y, const float z) 
{
	accelXLabel->setText(QString("Accel X: %1").arg(x));
	accelYLabel->setText(QString("Accel Y: %1").arg(y));
	accelZLabel->setText(QString("Accel Z: %1").arg(z));
}

void MainWindow::updateGPSState(const double lat, const double lon,
		const int satellites, const int gpsheading,
		const int alt, const int speed, const int fix)
{
	latLabel->setText(QString("Lat: %1").arg(lat));
	lonLabel->setText(QString("Lon: %1").arg(lon));
	satellitesLabel->setText(QString("Satellites: %1").arg(satellites));
	gpsheadingLabel->setText(QString("GPS Heading: %1").arg(gpsheading));
	fixLabel->setText(QString("Fix: %1").arg(fix));
	altLabel->setText(QString("Altitude: %1").arg(alt));
	groundspeedLabel->setText(QString("Ground Speed: %1").arg(speed));
}

void MainWindow::updateBatteryState(const int bat, const int current,
		const int amp)
{
	batLabel->setText(QString("Bat: %1").arg(bat));
	currentLabel->setText(QString("Current: %1").arg(current));
	ampLabel->setText(QString("Amps: %1").arg(amp));
}

void MainWindow::updateOPLinkStatus(const int rssi, const int quality,
		const int state)
{
	rssiLabel->setText(QString("RSSI: %1").arg(rssi));
	linkqualityLabel->setText(QString("Link Quality: %1").arg(quality));
	linkstateLabel->setText(QString("Link State: %1").arg(state));
}

void MainWindow::updateFlightStatus(const int armed, const int mode)
{
	armLabel->setText(QString("Armed: %1").arg(armed));
	flightmodeLabel->setText(QString("Flight Mode: %1").arg(mode));
}
