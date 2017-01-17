# TeensyXY
Dual axis ServoMotor code for Teensy 3.x 

/**********************************************************************************************
*  TeensyXY.ino    unix_guru at hotmail.com   @unix_guru on twitter
*  http://arduino-pi.blogspot.com
*
*  This sketch allows you to run two salvaged printer carriages for X/Y axis using their 
*  linear encoder strips for tracking on a Teensy 3.1 uController module based on Freescale's
*  Kinetis K20DX256 ARM Coretex-M4 processor with two hardware based Quadrature Decoder channels
*
*  The purpose of this sketch is to test and tune the PID loops for two AXIS simultaneously
*
*  This example uses the Arduino PID Library found at:
*  https://github.com/br3ttb/Arduino-PID-Library/archive/master.zip
*
*
* The most important part of this entire project however came from Trudy Benjamin with 
* her FTM based Quadrature Decoder library.
* https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x
*
* Thank you Trudy.
*
* Also thanks to Miguel Sanchez for helping me through some of the PID issues 
* and validating that this was a project worth doing.  Check out Miguel's Arduino DCServo work
* https://github.com/misan
* 
*
* Supports the typical Gcodes G0-G1, G28, G90, G91
*
* Using G150 for configuring Axis Position PID 
*     usage:  G150 X P10   or G150 Y P10 I2 D0.2
*
* Using G151 for configuring Axis Velocity PID 
*     usage:  G151 X P10   or G151 Y P10 I2 D0.2
*
* Using G160 to identify Axis limits (Left/right endstop positions, MAX Velocity/Accelleration)
* These limits will be populated into the EEPROM per axis.
*     usage: G160 X   --- Update X axis limits.
*
* Using G170 to reset current Axis position counter to Zero
*     usage G170 X    --- The current position of X becomes the new left endstop
*
* Using G180 to dump the most recent performance log to serial
*     usage G180      --- Most recent motion curve parameters will be dumped to serial for 
*                     --- Analysis.
*
* M500 to write config to EEPROM
* M501 to read config from EEPROM
*
************************************************************************************************/


