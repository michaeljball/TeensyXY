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

void updateDisplay() {            // update Serial output

   // For diagnostic  comment out later
    Serial.println("X: TPos, CPos, Velocity, PKp, Pki, Pkd "); 
    Serial.print("X, "); Serial.print(axis[0].ppid.tpos);Serial.print(", "); Serial.print(axis[0].ppid.pos);Serial.print(", "); Serial.print(axis[0].ppid.vel);
    Serial.print(", "); Serial.print(axis[0].ppid.Kp); Serial.print(", "); Serial.print(axis[0].ppid.Ki); Serial.print(", "); Serial.println(axis[0].ppid.Kd);
    Serial.println("X: TVel, CVel, PWM,    VKp, Vki, Vkd ");     
    Serial.print("X, "); Serial.print(axis[0].vpid.tvel);Serial.print(", "); Serial.print(axis[0].vpid.vel);Serial.print(", "); Serial.print(axis[0].vpid.spd);
    Serial.print(", "); Serial.print(axis[0].vpid.Kp); Serial.print(", "); Serial.print(axis[0].vpid.Ki); Serial.print(", "); Serial.println(axis[0].vpid.Kd);

    Serial.println();
    
    Serial.println("Y: TPos, CPos, Velocity, PKp, Pki, Pkd "); 
    Serial.print("Y, "); Serial.print(axis[1].ppid.tpos);Serial.print(", "); Serial.print(axis[1].ppid.pos);Serial.print(", "); Serial.print(axis[1].ppid.vel);
    Serial.print(", "); Serial.print(axis[1].ppid.Kp); Serial.print(", "); Serial.print(axis[1].ppid.Ki); Serial.print(", "); Serial.println(axis[1].ppid.Kd);
    Serial.println("Y: TVel, CVel, PWM,    VKp, Vki, Vkd ");     
    Serial.print("Y, "); Serial.print(axis[1].vpid.tvel);Serial.print(", "); Serial.print(axis[1].vpid.vel);Serial.print(", "); Serial.print(axis[1].vpid.spd);
    Serial.print(", "); Serial.print(axis[1].vpid.Kp); Serial.print(", "); Serial.print(axis[1].vpid.Ki); Serial.print(", "); Serial.println(axis[1].vpid.Kd);
    
    Serial.println();     
}

void dumpPerflog() {  
    Serial.printf("\r\n Log dump of most recent motion \r\n");
    Serial.printf("\r\nTime, Xpos, Xvel, Ypos, Yvel \r\n");
    
  for(int i=0;i<1000;i++) {
    Serial.print(i);Serial.print(", ");
    Serial.print(perflog[i].Xpos);Serial.print(", "); 
    Serial.print(perflog[i].Xvel);Serial.print(", ");
    Serial.print(perflog[i].Ypos);Serial.print(", ");
    Serial.println(perflog[i].Yvel); 
        
    perflog[i].Xpos=perflog[i].Xvel=perflog[i].Ypos=perflog[i].Yvel=0;    // Clear previous content    
  }
  logpos=0;
  logging = false;
}
