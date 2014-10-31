// AVR class for the MPL3115A2 altimeter
// Uses I2C 

#include "inc/MPL3115A2.h"
#include "inc/I2C.h"
#include "inc/UART.h"

MPL3115A2::MPL3115A2()
{
}

// Initializes the hardware and setups the values
uint8_t MPL3115A2::init(void)
{

	// Starting twi
	I2C::init();

	// sending start condition
	//if (I2C::start() != TW_START) 
	//	return 1;

	uint8_t whoami = readFromReg(MPL3115A2_WHOAMI);
	if (whoami != 0xC4) return 2;

	writeToReg(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_SBYB 
			| MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT);

	writeToReg(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
		   	MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM);

	return 0;
}

// Returns the measured pressure
float MPL3115A2::getPressure(void)
{
	return 0;
}

// Returns the altitude 
float MPL3115A2::getAltitude(void)
{
	//toggleOneShot();

	uint32_t alt;
	uint8_t counter = 0;
	while ((readFromReg(MPL3115A2_REGISTER_STATUS) & (1 << 1)) == 0)
	{
		if (++counter > 100) return -999; 
	}

	I2C::start();
	I2C::write((MPL3115A2_ADDRESS << 1 & 0xFE) | TW_WRITE);
	I2C::write(MPL3115A2_REGISTER_PRESSURE_MSB);
	I2C::start();
	I2C::write((MPL3115A2_ADDRESS << 1 & 0xFE) | TW_READ);
	alt = I2C::readAck();
	alt <<= 8;
	alt |= I2C::readAck();
	alt <<= 8;
	alt |= I2C::readNack();
	alt >>= 4;

	float altitude = alt;
	altitude /= 16.0;

	return altitude;
}

// Returns the temperature
float MPL3115A2::getTemperature(void)
{
	return 0;
}

void MPL3115A2::toggleOneShot() 
{
	uint8_t tempSetting = readFromReg(MPL3115A2_CTRL_REG1);
	tempSetting &= ~(1 << 1);
	writeToReg(MPL3115A2_CTRL_REG1, tempSetting);

	tempSetting = readFromReg(MPL3115A2_CTRL_REG1);
	tempSetting |= (1 << 1);
	writeToReg(MPL3115A2_CTRL_REG1, tempSetting);
}

uint8_t MPL3115A2::readFromReg(uint8_t addr) 
{
	I2C::start();
	I2C::write((MPL3115A2_ADDRESS << 1 & 0xFE) | TW_WRITE);
	I2C::write(addr);
	I2C::stop();
	I2C::start();
	I2C::write(((MPL3115A2_ADDRESS << 1) & 0xFE) | (TW_READ));
	uint8_t in = I2C::readNack();
	I2C::stop();
	return in;
}

void MPL3115A2::writeToReg(uint8_t addr, uint8_t data)
{
	I2C::start();
	I2C::write((MPL3115A2_ADDRESS << 1 & 0xFE) | TW_WRITE);
	I2C::write(addr);
	I2C::write(data);
	I2C::stop();
}
