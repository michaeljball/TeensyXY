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



/***************************************************************************************************************
*
*  ParseSerialData() allows the modification of the PID parameters as well as selecting new target 
*  
***************************************************************************************************************/

void ParseSerialData()
{
  char *p = inData;                // The data to be parsed
  char *str;                       // Temp store for each data chunk
  int count = 0;                   // Id ref for each chunk
    
  while ((str = strtok_r(p, ",", &p)) != NULL)    // Loop through the data and seperate it into chunks at each "," delimeter
  { 
    inParse[count] = str;      // Add chunk to array  
    count++;      
  }

  if(count > 1)     // If the data has two values then..  
  {
    // Define value 1 as a Command identifier
    char *Command = inParse[0];
    // Define value 2 as a Parameter value
    char *Parameter = inParse[1];

    Serial.print("CMD,"); Serial.print(Command); Serial.print(","); Serial.print(Parameter); Serial.print(","); Serial.println("0");
    
    // Call the relevant identified Command 
    switch(*Command)
    {

      case 'p':                                                         // Set 'Proportional'
        axis[0].vpid.Kp = atof(Parameter);       
        axis[1].vpid.Kp = atof(Parameter);       
        Serial.print("KpX = "); Serial.print(Parameter); Serial.print("\n\r");
        break;

      case 'i':                                                         // Set "Integral'
        axis[0].vpid.Ki = atof(Parameter);       
        axis[1].vpid.Ki = atof(Parameter);       
        Serial.print("KiX = "); Serial.print(Parameter); Serial.print("\n\r");;
        break;

      case 'd':                                                         // Set 'Derivative'
        axis[0].vpid.Kd = atof(Parameter);       
        axis[1].vpid.Kd = atof(Parameter);       
        Serial.print("KdX = "); Serial.print(Parameter); Serial.print("\n\r");
        break;

      case 'n':                                                         // New SP
        axis[0].ppid.tpos = atof(Parameter);       
        axis[1].ppid.tpos = atof(Parameter);       
        Serial.print("XaxisSP = "); Serial.print(Parameter); Serial.print("\n\r");
        break;

      case 's':                                                         // Set MAX Speed
        int Spd = atoi(Parameter);   
        xvPID.SetOutputLimits(0-Spd,Spd);           
        yvPID.SetOutputLimits(0-Spd,Spd);               
        Serial.print("Max Spd = "); Serial.print(Parameter); Serial.print("\n\r");
        break;
        
     } 
  } else {      // Else the data has a single value..  
    
    // Define value 1 as a Command identifier
    char *Command = inParse[0];

    Serial.print("CMD,"); Serial.println(Command); 
    
    // Call the relevant identified Commandt  
    switch(*Command)
    {

      case 'r':    
        mode = 0;   
        Serial.print("Mode = Random \n\r");
        break;
        
      case 'N':                                                         // Next State
        updateState();
        break;
        
      case 'h':                                                         // Print Help Display 
        Serial.println("Inputs can be p, i, d, s, r, l, N");
        break;        
    }
  }
}


void SerialEvent2() 
{
  while (Serial.available() && stringComplete == false)    // Read while we have data
  {
    char inChar = Serial.read();             // Read a character

    inData[chindex] = inChar;                  // Store it in char array
    chindex++;                                 // Increment where to write next  
    inString += inChar;                      // Also add it to string storage just in case, not used yet :)
    
    if (inChar == '\n' || inChar == '\r')                      // Check for termination character
    {
      chindex = 0;
      stringComplete = true;
    }
  }
}
