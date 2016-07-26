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


// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
// G30 - Single Z Probe, probes bed at current XY location.
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set  - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
//        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
// M112 - Emergency stop
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homing offset
// M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beep sound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from EEPROM)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M665 - set delta configurations
// M666 - set delta endstop adjustment
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error




//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 16

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;


static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
char command[MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;             // Buffer Read Index
static int bufindw = 0;             // Buffer Write Index
static int buflen = 0;              // Number of Commands in the Buffer
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y etc

static bool Stopped = false;
static bool relative_mode = false;
static long  previous_millis_cmd;

#define NUM_AXIS 2 // The axis order in all axis related arrays is X, Y
const bool AXIS_RELATIVE_MODES[2]= {false, false};
const char axis_codes[NUM_AXIS]= {'X', 'Y'};
enum AxisEnum {X_AXIS=0, Y_AXIS=1};

float current_position[NUM_AXIS];
float destination[NUM_AXIS];

const char pid_codes[3]= {'P', 'I', 'D'};


#define PROTOCOL_VERSION "1.0"
#define MACHINE_NAME "PiBot 1.0"
#define FIRMWARE_URL "https://github.com/michaeljball"
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#define MSG_M115_REPORT "FIRMWARE_NAME:XYTeensy V1; Sprinter/grbl mashup  FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"


void get_buffer()                                   // Process serial input to fill command buffer
{
    if(buflen < (BUFSIZE-1))  get_command();
   
    if(buflen)
    {

        process_commands();
        buflen = (buflen-1);
        bufindr = (bufindr + 1)%BUFSIZE;
      
   }
}    

// Read the serial port (if available, and add character to the end of the current command string
// intil either a CR, LF, or ':' is seen or maximum string size is reached.. 
// Then push string onto cmdbuffer[] queue.
void get_command()                  
{
  while( Serial.available() > 0  && buflen < BUFSIZE) {
    serial_char = Serial.read();
 
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) {                                               //if empty line
        comment_mode = false;                                           //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0;                             //terminate string
      if(!comment_mode){
        comment_mode = false;                                           //for new command

        if(strchr(cmdbuffer[bufindw], 'N') != NULL) {                   // Validate checksum and line number of input string
                 
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {           // Set Line Number
    
            Serial.printf("Error: on Line # %ul ", gcode_LastN);
            Serial.printf("%ul\r\n",gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL) {
            char checksum = 0;
            char count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              Serial.printf("Error: Checksum Error on line # %ul\r\n",gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          } else {
            Serial.printf("Error: No Checksum on line # %ul\r\n",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;                                        // Update line number 
          //if no errors, continue parsing
           
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            Serial.printf("Error: No Line Number on line # %ul\r\n",gcode_LastN);

            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){               // 
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
              Serial.printf("ok\r\n");
            }
            else {
              Serial.printf("Error: STOPPED\r\n");
            }
            break;
          default:
            break;
          }

        }

        if(strcmp(cmdbuffer[bufindw], "M112") == 0)  kill();             //If command was e-stop process now
        
        strcpy(command,cmdbuffer[bufindw]);
        bufindw = (bufindw + 1)%BUFSIZE;        // Current string completed, Point to the next string buffer
        buflen += 1;                            // Increment the number of items in the queue
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;   // Add current character to the string
    }
    
  }
}


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void FlushSerialRequestResend()
{

  char char1 = 0; 
  while (Serial.available()) char1 = Serial.read();              // Flush the serial buffer
  Serial.printf("Resend: %ul\r\n", (gcode_LastN + 1));
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  Serial.printf("ok\r\n");
  delay(10);
}


// This routine actually parses the G/M codes, and executes functions based on input
void process_commands()
{
 unsigned long codenum; //throw away variable
 char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E

        prepare_move();
        ClearToSend();
        return;
      }
      break; 
    case 28: // G28       Home all axis
            axis[0].ppid.tpos = 0;
            axis[1].ppid.tpos = 0;  
      break;
    case 90: // G90
      relative_mode = false;    
      break;
    case 91: // G91
      relative_mode = true;     
      break;
    case 92: // G92   Set to absolute position

      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
          current_position[i] = code_value();
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
        }
      }   
    break;     
      
    case 150: // G150   Co-Opting to Set Position PID  P
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {        // Select which Axis to update
          if (code_seen('P')) axis[i].ppid.Kp = code_value();
          if (code_seen('I')) axis[i].ppid.Ki = code_value();
          if (code_seen('D')) axis[i].ppid.Kd = code_value();
          Serial.printf("Axis %d PKp %f PKp %f PKp %f \r\n", i, axis[i].ppid.Kp,axis[i].ppid.Ki,axis[i].ppid.Kd);          
        }  
      }   
      break;     
      

    case 160: // G150   Co-Opting to Set Position PID  P
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {        // Select which Axis to update
          if (code_seen('P')) axis[i].vpid.Kp = code_value();
          if (code_seen('I')) axis[i].vpid.Ki = code_value();
          if (code_seen('D')) axis[i].vpid.Kd = code_value();
          Serial.printf("Axis %d PKp %f PKp %f PKp %f \r\n", i, axis[i].ppid.Kp,axis[i].ppid.Ki,axis[i].ppid.Kd);          
        }  
      }   
      break;     

    case 170: // G170   Reset Encoder Counts to Zero
      xPosn.zeroFTM(); yPosn.zeroFTM();                 // Start Quad Decode position count   
      Serial.printf("X/Y Axis bot reset to Zero\r\n");                     
      break;

    case 180: // G180   Dump Performance log to serial
      dumpPerflog();
      break;
      
    };
   }
    else if(code_seen('M'))
    {
        switch( (int)code_value() )
        {
          case 112: //  M112 -Emergency Stop
            kill();
          break;
          
          case 105 : // M105
                Serial.printf("Not supported");
           break;
          case 115: // M115
                Serial.printf("FIRMWARE_NAME: TeensyXY V1; DC ServoMotor CTRL  ");
                Serial.printf("FIRMWARE_URL: %s PROTOCOL_VERSION: %s  MACHINE_TYPE: %s ", FIRMWARE_URL, PROTOCOL_VERSION, MACHINE_NAME );
                Serial.printf("EXTRUDER_COUNT: 1 UUID: %s \r\n", MACHINE_UUID);

          break; 

          case 500 : // M500   Save parameters to EEPROM
                writeConfig();
                Serial.printf("Parameters written to EEPROM");
           break;          
          case 501 : // M501   Read parameters from EEPROM
                readConfig();
                Serial.printf("Parameters updated from EEPROM");
           break;
        }
    }   
     else
        {
            Serial.printf("echo: Unknown Command %s \r\n",cmdbuffer[bufindr]);
        }

    ClearToSend();
}      


// Stub functions for now... ******************************************************************************


void get_coordinates()
{
    bool seen[2]= {false,false};
    for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
            axis[i].ppid.tpos = (float)code_value() + (AXIS_RELATIVE_MODES[i] || relative_mode)*axis[i].ppid.pos;
            seen[i]=true;
            logging = true;
            Serial.printf("Destination %u = %f  \r\n", i, axis[i].ppid.tpos);             // ******************************************* DEBUG  ***************************8
        } 
    }

}

void prepare_move(void) {
  
}




void plan_set_position(double X, double Y){
            axis[0].ppid.tpos = X;
            axis[1].ppid.tpos = Y;            
  
}

void kill(void){

    axis[0].vpid.spd= 0;
    axis[0].vpid.spd= 0;  
}

