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

// Thanks to Scott Penrose for the following http://linux.dd.com.au/wiki/Teensy_EEPROM_Logger



#define LOG_START 50      // Start EEPROM
#define LOG_SIZE 1900     // Size (bytes) allowed (max log size)



void readConfig() {
  int ConfigLoc=0;
  time_t configTime;
  EEPROM.readBlock( 0, configTime );
  ConfigLoc += sizeof(time_t);
  EEPROM.readBlock(ConfigLoc, axis[0]); 
  ConfigLoc += sizeof(axis[0]);
  EEPROM.readBlock(ConfigLoc, axis[1]); 
}

void writeConfig() {
  int ConfigLoc=0;
  time_t timeNow = now();
  EEPROM.writeBlock(ConfigLoc, timeNow );
  ConfigLoc += sizeof(timeNow);
  EEPROM.writeBlock(ConfigLoc, axis[0]); 
  ConfigLoc += sizeof(axis[0]);
  EEPROM.writeBlock(ConfigLoc, axis[1]); 
}



void report_ram_stat(const char* aname, uint32_t avalue) {
  Serial.print(aname);
  Serial.print(": ");
  Serial.print((avalue + 512) / 1024);
  Serial.print(" Kb (");
  Serial.print((((float) avalue) / ram.total()) * 100, 1);
  Serial.println("%)");
};

void report_ram() {
  bool lowmem;
  bool crash;
  
  Serial.println("==== memory report ====");
  
  report_ram_stat("free", ram.adj_free());
  report_ram_stat("stack", ram.stack_total());
  report_ram_stat("heap", ram.heap_total());
  
  lowmem = ram.warning_lowmem();
  crash = ram.warning_crash();
  if(lowmem || crash) {
    Serial.println();
    
    if(crash)
      Serial.println("**warning: stack and heap crash possible");
    else if(lowmem)
      Serial.println("**warning: unallocated memory running low");
  };
  
  Serial.println();
};
