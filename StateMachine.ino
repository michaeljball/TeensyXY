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
************************************************************************************************/


/*  The State machine is for demonstrating target acquisition from an array prior 
*   to getting GCode parsder functional.
*   
*   Ok... *THIS* version is static... whatever..
*
*/

void updateState() {            // update LCD readout

      Serial.printf("Current State is %l\r\n",currentState);
      switch(currentState) {
        case 0:
          axis[0].ppid.tpos = 1500;       
          axis[1].ppid.tpos = 0;       
          currentState=1;
          break;
          
        case 1:
          axis[0].ppid.tpos = 1500;       
          axis[1].ppid.tpos = 1500;        ;
          currentState=2;
          break;
          
        case 2:
          axis[0].ppid.tpos = 0;       
          axis[1].ppid.tpos = 1500;
          currentState=3;
          break;
          
        case 3:
          axis[0].ppid.tpos = 0;       
          axis[0].ppid.tpos = 0;       
          currentState=0;
          break;
      }            
}
