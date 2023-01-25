/******************************************************************************
SparkFunLIS3DH.h
LIS3DH Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
Nov 16, 2016
https://github.com/sparkfun/LIS3DH_Breakout
https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LIS3DH_IMU_H__
#define __LIS3DH_IMU_H__

#include "stdint.h"

//values for commInterface
#define I2C_MODE 0
#define SPI_MODE 1

// Return values 
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

// This is the core operational class of the driver.
//   LIS3DHCore contains only read and write operations towards the IMU.
//   To use the higher level functions, use the class LIS3DH which inherits
//   this class.

class LIS3DHCore
{
public:
	LIS3DHCore(uint8_t);
	LIS3DHCore(uint8_t, uint8_t);
	~LIS3DHCore() = default;

	status_t beginCore(void);

	// The following utilities read and write to the IMU

	// ReadRegisterRegion takes a uint8 array address as input and reads
	//   a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t *, uint8_t, uint8_t);

	// readRegister reads one 8-bit register
	status_t readRegister(uint8_t *, uint8_t);

	// Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//   Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t *, uint8_t offset);

	// Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);

private:
	// Communication stuff
	uint8_t commInterface;
	uint8_t I2CAddress;
	uint8_t chipSelectPin;
};

// This struct holds the settings the driver uses to do calculations
struct SensorSettings
{
public:
	// ADC and Temperature settings
	uint8_t adcEnabled;
	uint8_t tempEnabled;

	// Accelerometer settings
	uint16_t accelSampleRate; // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	uint8_t accelRange;		  // Max G force readable.  Can be: 2, 4, 8, 16

	uint8_t xAccelEnabled;
	uint8_t yAccelEnabled;
	uint8_t zAccelEnabled;

	// Fifo settings
	uint8_t fifoEnabled;
	uint8_t fifoMode; // can be 0x0,0x1,0x2,0x3
	uint8_t fifoThreshold;
};

// This is the highest level class of the driver.
//
//   class LIS3DH inherits the core and makes use of the beginCore()
// method through it's own begin() method.  It also contains the
// settings struct to hold user settings.

class LIS3DH : public LIS3DHCore
{
public:
	// IMU settings
	SensorSettings settings;

	// Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

	// Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
	LIS3DH(uint8_t busType = I2C_MODE, uint8_t inputArg = 0x19);
	//~LIS3DH() = default;

	// Call to apply SensorSettings
	status_t begin(void);
	void applySettings(void);

	// Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX(void);
	int16_t readRawAccelY(void);
	int16_t readRawAccelZ(void);

	// Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX(void);
	float readFloatAccelY(void);
	float readFloatAccelZ(void);

	// ADC related calls
	uint16_t read10bitADC1(void);
	uint16_t read10bitADC2(void);
	uint16_t read10bitADC3(void);

	// FIFO stuff
	void fifoBegin(void);
	void fifoClear(void);
	uint8_t fifoGetStatus(void);
	void fifoStartRec();
	void fifoEnd(void);

	float calcAccel(int16_t);

private:
};

// Device Registers - these are in order of how they appear in the datasheet table 21
#define LIS3DH_STATUS_REG_AUX 0x07
#define LIS3DH_OUT_ADC1_L 0x08
#define LIS3DH_OUT_ADC1_H 0x09
#define LIS3DH_OUT_ADC2_L 0x0A
#define LIS3DH_OUT_ADC2_H 0x0B
#define LIS3DH_OUT_ADC3_L 0x0C
#define LIS3DH_OUT_ADC3_H 0x0D
#define LIS3DH_WHO_AM_I 0x0F

#define LIS3DH_CTRL_REG0 0x1E
#define LIS3DH_TEMP_CFG_REG 0x1F
#define LIS3DH_CTRL_REG1 0x20
#define LIS3DH_CTRL_REG2 0x21
#define LIS3DH_CTRL_REG3 0x22
#define LIS3DH_CTRL_REG4 0x23
#define LIS3DH_CTRL_REG5 0x24
#define LIS3DH_CTRL_REG6 0x25
#define LIS3DH_REFERENCE 0x26
#define LIS3DH_STATUS_REG 0x27

#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_L 0x2C
#define LIS3DH_OUT_Z_H 0x2D

#define LIS3DH_FIFO_CTRL_REG 0x2E
#define LIS3DH_FIFO_SRC_REG 0x2F

#define LIS3DH_INT1_CFG 0x30
#define LIS3DH_INT1_SRC 0x31
#define LIS3DH_INT1_THS 0x32
#define LIS3DH_INT1_DURATION 0x33

#define LIS3DH_INT2_CFG 0x34
#define LIS3DH_INT2_SRC 0x35
#define LIS3DH_INT2_THS 0x36
#define LIS3DH_INT2_DURATION 0x37

#define LIS3DH_CLICK_CFG 0x38
#define LIS3DH_CLICK_SRC 0x39
#define LIS3DH_CLICK_THS 0x3A
#define LIS3DH_TIME_LIMIT 0x3B
#define LIS3DH_TIME_LATENCY 0x3C
#define LIS3DH_TIME_WINDOW 0x3D

#define LIS3DH_ACT_THS 0x3E
#define LIS3DH_ACT_DUR 0x3F

/// @brief These are all helper enums so we can use the wording in our code, and do binary operations to get to the right value.
struct LIS3DHEnums
{
	enum CTRL_REG0
	{
		PullUpConnected    = 0b00010000,
		PullUpDisconnected = 0b10010000
	};

	enum TEMP_CFG_REG
	{
		ADC_TEMP_ENABLED = 0b11000000,
		ADC_ENABLED      = 0b10000000,
		TEMP_ENABLED     = 0b01000000,
		ALL_DISABLED     = 0b00000000
	};

	enum CTRL_REG1
	{
		ORD3 = 0b10000000,
		ORD2 = 0b01000000,
		ORD1 = 0b00100000,
		ORD0 = 0b00010000,
		LPen = 0b00001000,
		Zen  = 0b00000100,
		Yen  = 0b00000010,
		Xen  = 0b00000001,
		XYZen = 0b00000111, /* Shortcut */
	};

	enum CTRL_REG2
	{
		HPM1 = 0b10000000,
		HPM2 = 0b01000000,
		HPCF2 = 0b00100000,
		HPCF1 = 0b00010000,
		FDS = 0b00001000,
		HPCLICK = 0b00000100,
		HP_IA2 = 0b00000010,
		HP_IA1 = 0b00000001,
	};
	/// @brief CTRL_REG3 controls where interupt 1 is send to.
	enum CTRL_REG3
	{
		I1_CLICK = 0b10000000,
		I1_IA1 = 0b01000000,
		I1_IA2 = 0b00100000,
		I1_ZYXDA = 0b00010000,
		I1_321DA = 0b00001000,
		I1_WTM = 0b00000100,
		I1_OVERRUN = 0b00000010,
	};

	enum CTRL_REG4
	{
		BDU          = 0b10000000,
		BLE          = 0b01000000,
		FS_2G        = 0b00000000,
		FS_4G        = 0b00010000,
		FS_8G        = 0b00100000,
		FS_16G       = 0b00110000,
		HR           = 0b00001000,
		ST_NORMAL    = 0b00000000,
		ST_SelfTest0 = 0b00000010,
		ST_SelfTest1 = 0b00000100,
		SIM          = 0b00000001,
	};

	enum CTRL_REG5
	{
		BOOT = 0b10000000,
		FIFO_EN = 0b01000000,
		LIR_INT1 = 0b00001000,
		D4D_INT1 = 0b00000100,
		LIR_INT2 = 0b00000010,
		D4D_INT2 = 0b00000001,
	};

	/// @brief CTRL_REG6 controls where interupt 2 is send to.
	enum CTRL_REG6
	{
		I2_CLICK = 0b10000000,
		I2_IA1 = 0b01000000,
		I2_IA2 = 0b00100000,
		I2_BOOT = 0b00010000,
		I2_ACT = 0b00001000,
		INT_POLARITY = 0b00000010,
	};

	enum STATUS_REG
	{
		ZYXOR = 0b10000000,
		ZOR = 0b01000000,
		YOR = 0b00100000,
		XOR = 0b00010000,
		ZYXDA = 0b00001000,
		ZDA = 0b00000100,
		YDA = 0b00000010,
		XDA = 0b00000001,
	};

	enum STATUS_REG_AUX
	{
		_321OR = 0b10000000,
		_3OR = 0b01000000,
		_2OR = 0b00100000,
		_1OR = 0b00010000,
		_321DA = 0b00001000,
		_3DA = 0b00000100,
		_2DA = 0b00000010,
		_1DA = 0b00000001,
	};

	enum INT_CFG
	{
		AOI = 0b10000000,
		_6D = 0b01000000,
		ZHIE = 0b00100000,
		ZLIE = 0b00010000,
		YHIE = 0b00001000,
		YLIE = 0b00000100,
		XHIE = 0b00000010,
		XLIE = 0b00000001,
	};

	enum INT_SRC {
		IA   = 0b01000000,
		ZH   = 0b00100000,
		ZL   = 0b00010000,
		YH   = 0b00001000,
		YL   = 0b00000100,
		XH   = 0b00000010,
		XL   = 0b00000001,
	};
};

#endif // End of __LIS3DH_IMU_H__ definition check
