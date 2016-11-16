SparkFun LIS3DH Arduino Library
========================================

**BROKEN** ![LIS3DH Breakout](https://www.sparkfun.com/)

**BROKEN** [*LIS3DH Breakout (SEN-XXXXX)*](https://www.sparkfun.com/)

This is an arduino IDE library to control the LIS3DH.  It can be configured to use I2C or SPI with 2 instances per I2C channel or any number of SPI instances.  The top-level driver, class LIS3DH, contains an inner driver LIS3DHCore, a settings struct, and float-based math functions for conversion from raw to meaningful numbers.

This has been tested with Arduino Uno.  TODO:(*and Teensy 3.1 architectures*).

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/extras** - Contains class diagrams for the driver.  Ignored by IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Example Briefs
--------------

* ADCUsage - Demonstrates analog in reads and has notes about temperature collection
* FifoExample - Demonstrates using the built-in buffer to burst-collect data - **Good demonstration of settings**
* FullSettingExample - Shows all settings, with non-used options commented out
* IntUsage - shows configuration of interrupt bits
* LowLevelExample - Demonstrates using only the core driver without math and settings overhead
* MinimalistExample - The **easiest** configuration
* MultiI2C - Using two LIS3DHs over I2C
* MultiSPI - Using two LIS3DHs over SPI

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **BROKEN** **[Product Repository](https://www.sparkfun.com/)** - Main repository (including hardware files) for the LIS3DH Breakout.
* **BROKEN** **[Hookup Guide](https://www.sparkfun.com/)** - Basic hookup guide for the LIS3DH Breakout.

Products that use this Library 
---------------------------------

* [SEN-XXXXX](https://www.sparkfun.com/)- LIS3DH Breakout board

Version History
---------------

* **BROKEN**[V 1.0.0](https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library/releases/tag/V_1.0.0) -- Initial commit of Arduino 1.6-compatible library.

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

