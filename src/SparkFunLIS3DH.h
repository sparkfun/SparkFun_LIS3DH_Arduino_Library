/******************************************************************************
SparkFunLIS3DH.h
LIS3DH Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
May 20, 2015
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

//This is the core operational class of the driver.
//  LIS3DHCore contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LIS3DH which inherits
//  this class.

class LIS3DHCore
{
public:
	LIS3DHCore( uint8_t );
	LIS3DHCore( uint8_t, uint8_t );
	~LIS3DHCore() = default;
	
	status_t beginCore( void );
	
	//The following utilities read and write to the IMU

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	
	//readRegister reads one 8-bit register
	status_t readRegister(uint8_t*, uint8_t);
	
	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t offset );
	
	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);
	
	
private:
	
	//Communication stuff
	uint8_t commInterface;
	uint8_t I2CAddress;
	uint8_t chipSelectPin;

};

//This struct holds the settings the driver uses to do calculations
struct SensorSettings {
public:

	uint8_t adcEnabled;
	
	//Temperature settings
	uint8_t tempEnabled;

	//Accelerometer settings
	uint16_t accelSampleRate;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	uint8_t accelRange;      //Max G force readable.  Can be: 2, 4, 8, 16

	uint8_t xAccelEnabled;
	uint8_t yAccelEnabled;
	uint8_t zAccelEnabled;
	
	////ADC stuff
	//uint8_t adcEnabled;
	//
	////Temperature settings
	//uint8_t tempEnabled;
    //
	////Accelerometer settings
	//uint8_t accelEnabled;
	//uint16_t accelSampleRate;
	//uint8_t accelODROff;
	//uint16_t accelRange;
	//uint16_t accelBandWidth;
	//
	//uint8_t accelFifoEnabled;
	//uint8_t accelFifoDecimation;
	//
	//
	////Non-basic mode settings
	//uint8_t commMode;
	//
	////FIFO control data
	//uint16_t fifoThreshold;
	//int16_t fifoSampleRate;
	//uint8_t fifoModeWord;
	
};


//This is the highest level class of the driver.
//
//  class LIS3DH inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LIS3DH : public LIS3DHCore
{
public:
	//IMU settings
	SensorSettings settings;
	
	//Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
	LIS3DH( uint8_t busType = I2C_MODE, uint8_t inputArg = 0x19 );
	~LIS3DH() = default;
	
	//Call to apply SensorSettings
	status_t begin(void);

	//Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX( void );
	int16_t readRawAccelY( void );
	int16_t readRawAccelZ( void );

	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX( void );
	float readFloatAccelY( void );
	float readFloatAccelZ( void );

	//Temperature related methods
	int16_t readRawTemp( void );
	float readTempC( void );
	float readTempF( void );

	//FIFO stuff
	//void fifoBegin( void );
	//void fifoClear( void );
	//int16_t fifoRead( void );
	//uint16_t fifoGetStatus( void );
	//void fifoEnd( void );
	//
	//float calcGyro( int16_t );
	float calcAccel( int16_t );
	
private:

};






//Device Registers
#define LIS3DH_STATUS_REG_AUX         0x07
#define LIS3DH_OUT_ADC1_L             0x08
#define LIS3DH_OUT_ADC1_H             0x09
#define LIS3DH_OUT_ADC2_L             0x0A
#define LIS3DH_OUT_ADC2_H             0x0B
#define LIS3DH_OUT_ADC3_L             0x0C
#define LIS3DH_OUT_ADC3_H             0x0D
#define LIS3DH_INT_COUNTER_REG        0x0E
#define LIS3DH_WHO_AM_I               0x0F

#define LIS3DH_TEMP_CFG_REG           0x1F
#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SOURCE            0x31
#define LIS3DH_INT1_THS               0x32
#define LIS3DH_INT1_DURATION          0x33

#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

//Example enum:

//typedef enum {
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO 		 = 0x00,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION 		 = 0x01,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2 		 = 0x02,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3 		 = 0x03,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4 		 = 0x04,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8 		 = 0x05,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16 		 = 0x06,
//	LIS3DH_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32 		 = 0x07,
//} LIS3DH_ACC_GYRO_DEC_FIFO_XL_t;

#endif  // End of __LIS3DH_IMU_H__ definition check
