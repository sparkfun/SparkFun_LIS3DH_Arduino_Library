/******************************************************************************
MinimalistExample.ino

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
Most basic example of use.

Example using the LSM6DS3 with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"

LIS3DH myIMU(I2C_MODE, 0x19); //Default constructor is I2C, addr 0x19.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  myIMU.settings.adcEnabled = 0;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;
  
  //Call .begin() to configure the IMU
  myIMU.begin();
  
  uint8_t dataToWrite;  //Temporary variable
  
  //Configure interrupts for pin 1
  dataToWrite = 0;
  dataToWrite |= 0x80; //Click
  //dataToWrite |= 0x10; //Data ready
  //dataToWrite |= 0x04; //FIFO watermark
  //dataToWrite |= 0x02; //FIFO overrun
  dataToWrite |= 0x40; //enable outputs on pin 1
  myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
  
  //Configure interrupts for pin 2
  dataToWrite = 0;
  dataToWrite |= 0x80; //Click
  dataToWrite |= 0x10; //boot status
  //dataToWrite |= 0x02; //invert pin 2 output logic
  dataToWrite |= 0x40; //enable outputs on pin 2
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);

}


void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  delay(1000);
}
