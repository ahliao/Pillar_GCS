// AVR class for the MPL3115A2 altimeter
// Uses I2C 

#ifndef _MPL3115A2_H
#define _MPL3115A2_H

#include <stdint.h>

// I2C Address/BITS
#define MPL3115A2_ADDRESS (0x60) // 1100000

// Registers
#define MPL3115A2_REGISTER_STATUS (0x00)
#define MPL3115A2_REGISTER_STATUS_TDR 0x02
#define MPL3115A2_REGISTER_STATUS_PDR 0x04
#define MPL3115A2_REGISTER_STATUS_PTDR 0x08
#define MPL3115A2_REGISTER_PRESSURE_MSB (0x01)
#define MPL3115A2_REGISTER_PRESSURE_CSB (0x02)
#define MPL3115A2_REGISTER_PRESSURE_LSB (0x03)
#define MPL3115A2_REGISTER_TEMP_MSB (0x04)
#define MPL3115A2_REGISTER_TEMP_LSB (0x05)
#define MPL3115A2_REGISTER_DR_STATUS (0x06)
#define MPL3115A2_OUT_P_DELTA_MSB (0x07)
#define MPL3115A2_OUT_P_DELTA_CSB (0x08)
#define MPL3115A2_OUT_P_DELTA_LSB (0x09)
#define MPL3115A2_OUT_T_DELTA_MSB (0x0A)
#define MPL3115A2_OUT_T_DELTA_LSB (0x0B)
#define MPL3115A2_WHOAMI (0x0C)
#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_PT_DATA_CFG_TDEFE 0x01
#define MPL3115A2_PT_DATA_CFG_PDEFE 0x02
#define MPL3115A2_PT_DATA_CFG_DREM 0x04
#define MPL3115A2_CTRL_REG1 (0x26)
#define MPL3115A2_CTRL_REG1_SBYB 0x01
#define MPL3115A2_CTRL_REG1_OST 0x02
#define MPL3115A2_CTRL_REG1_RST 0x04
#define MPL3115A2_CTRL_REG1_OS1 0x00
#define MPL3115A2_CTRL_REG1_OS2 0x08
#define MPL3115A2_CTRL_REG1_OS4 0x10
#define MPL3115A2_CTRL_REG1_OS8 0x18
#define MPL3115A2_CTRL_REG1_OS16 0x20
#define MPL3115A2_CTRL_REG1_OS32 0x28
#define MPL3115A2_CTRL_REG1_OS64 0x30
#define MPL3115A2_CTRL_REG1_OS128 0x38
#define MPL3115A2_CTRL_REG1_RAW 0x40
#define MPL3115A2_CTRL_REG1_ALT 0x80
#define MPL3115A2_CTRL_REG1_BAR 0x00
#define MPL3115A2_CTRL_REG2 (0x27)
#define MPL3115A2_CTRL_REG3 (0x28)
#define MPL3115A2_CTRL_REG4 (0x29)
#define MPL3115A2_CTRL_REG5 (0x2A)
#define MPL3115A2_REGISTER_STARTCONVERSION (0x12)

class MPL3115A2{
	public:
		MPL3115A2();

		// Initializes the hardware and setups the values
		uint8_t init(void);

		// Returns the measured pressure
		float getPressure(void);

		// Returns the altitude 
		float getAltitude(void);

		// Returns the temperature
		float getTemperature(void);
	private:
		void toggleOneShot();

		void standBy();
		void active();

		uint8_t readFromReg(uint8_t a);

		void writeToReg(uint8_t a, uint8_t d);
};

#endif
