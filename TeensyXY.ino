#include <QuadDecode_def.h>
#include <QuadDecode.h>



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

#include <EEPROMex.h>

// #include <QuadDecode.h>
// #include <QuadDecode_Def.h>

// wire for Teensy 3.1 as per https://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3
#include <i2c_t3.h>  
#include <PID_v1.h> 

#include <stdbool.h>
#include <cstdint>

#include "RamMonitor.h"

#include <Time.h>  //  https://github.com/PaulStoffregen/Time    Teensy RTC



RamMonitor ram;                   // Manages free memory on uController 

time_t getTeensy3Time() { return Teensy3Clock.get(); }


#define frontstop  100            // Right most encoder boundary (allowing for some cushion)
#define backstop  7700            // Left most encoder boundary

#define XaxisPWM    23            //  PWM pin to drive X-Axis Motor
#define XaxisDir    11            //  Direction control for X-Axis
#define XaxisEND    5             //  Endstop for X-Axis (as yet unused)

#define YaxisPWM    22            //  PWM pin to drive Y-Axis Motor
#define YaxisDir    13            //  Direction control for Y-Axis
#define YaxisEND    8             //  Endstop for Y-Axis (as yet unused)

/********************************************************************************************
*  The Teensy Freescale FTM Quadrature Decoder channels are tied to specific pins 
*
*  X-Axis Quadrature Encoder Phase A is connected to Teensy 3.1 pin 3  using FTM1
*  X-Axis Quadrature Encoder Phase B is connected to Teensy 3.1 pin 4  using FTM1
*  Y-Axis Quadrature Encoder Phase A is connected to Teensy 3.1 pin 32 using FTM2
*  Y-Axis Quadrature Encoder Phase B is connected to Teensy 3.1 pin 25 using FTM2
*
**********************************************************************************************/

QuadDecode<1> xPosn;                            // Motor Encoder 1 Template using FTM1
QuadDecode<2> yPosn;                            // Motor Encoder 2 Template using FTM2



#define FORWARD      0
#define BACKWARD     1


#define PID_UPDATE_TIME   1000                 // 10 mSec update  (100 times a second)
IntervalTimer pidTimer;                         // How often to update state machine


void updatePID(void);                           // PID timing loop interrupt routine
void ParseSerialData(void);                     // Parsing Serial string for commands
void SerialEvent2(void);                        // Manage Serial buffer 

volatile int32_t rtX=0, rtY=0;                  // Realtime values of X,Y in encoder ticks

elapsedMillis doOutput;                         // Timing on printing to Serial

volatile byte currentState=0;                   // Initial Current State
volatile bool mode = 0;                         // Random or linear test
int linearStep = 2;                             // How far to travel between steps

int maxPWM = 255;                               // Maximum value for motor PWM
int maxVel = 1000;                              // Maximum velocity per axis in ticks/s
int maxAvailVel[] = {0,0};                      // Calculated Maximum available Velocity per Axis in ticks/s
double endstop[] = {0,0};                      // Position of the Right Endstop 

struct POSPIDSTRUCT {                         // Position PID structure
  float  Kp;                                  // Proportional Gain
  float  Ki;                                  // Integral Gain
  float  Kd;                                  // Differential Gain
  double pos;                                 // Current Position (PID Input)
  double vel;                                 // Current Velocity (PID Output)
  double tpos;                                // Target Position  (PID Setpoint) 
};

struct VELPIDSTRUCT {                         // Velocity PID structure
  float  Kp;                                  // Proportional Gain
  float  Ki;                                  // Integral Gain
  float  Kd;                                  // Differential Gain
  double vel;                                 // Current Velocity    (PID Input)
  double spd;                                 // Current Motor Speed (PID Output)
  double tvel;                                // Target Velocity     (PID Setpoint) 
};


struct AXISSTRUCT {                           // Structure to manage axis variables  
  POSPIDSTRUCT ppid = { 6, 0, 0, 0, 0, 0 };   // Initialize Position PID
  VELPIDSTRUCT vpid = { 3, 0, 0, 0, 0, 0 };   // Initialize Velocity PID
  int maxPWM = 180;                           // Maximum PWM for motor drive
  int maxVEL = 1000;                          // Maximum Velocity in encoder ticks/s 
  int maxACC = 5000;                          // Maximum Acceleration in encoder ticks/s/s
} axis[2];                                    // Instantiate two axis

struct AXISLOG {                              // Structure to log PID performance for tuning
  double Xpos;                                // Current X position in encoder ticks
  double Xvel;                                // Current X velocity in ticks/second
  double Ypos;                                // Current Y position in encoder ticks
  double Yvel;                                // Current Y velocity in ticks/second  
} perflog[1000];

int  logpos = 0;                              // Position in the motion 
bool logging = false;                         // Not currently logging
bool motion = false;                          // Not currently moving

// Instantiate X and Y axis PID controls
PID xpPID(&axis[0].ppid.pos, &axis[0].ppid.vel, &axis[0].ppid.tpos, axis[0].ppid.Kp, axis[0].ppid.Ki, axis[0].ppid.Kd, DIRECT);        // PID controller for X axis Position
PID xvPID(&axis[0].vpid.vel, &axis[0].vpid.spd, &axis[0].vpid.tvel, axis[0].vpid.Kp, axis[0].vpid.Ki, axis[0].vpid.Kd, DIRECT);        // PID controller for X axis Position

PID ypPID(&axis[1].ppid.pos, &axis[1].ppid.vel, &axis[1].ppid.tpos, axis[1].ppid.Kp, axis[1].ppid.Ki, axis[1].ppid.Kd, DIRECT);        // PID controller for X axis Position
PID yvPID(&axis[1].vpid.vel, &axis[1].vpid.spd, &axis[1].vpid.tvel, axis[1].vpid.Kp, axis[1].vpid.Ki, axis[1].vpid.Kd, DIRECT);        // PID controller for X axis Position

const int sampleRate = 1;                 // Calling compute() from a timer interrupt.

char disbuffer[32];        // Used for formatting numbers to display


// ================================== Variables related to Command Processing ==============================================================
        
char Command = 's';
int Parameter = 0;

char inData[64];                                           // Buffer for the incoming data
char *inParse[64];                                         // Buffer for the parsed data chunks

String inString = "";                                      // Storage for data as string
int chindex = 0;
boolean stringComplete = false;


// ================================== Variables related to Motion Planner ==============================================================


// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
// long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  long steps[4];                            // Step count for each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float speed[4];                                    // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  volatile char busy;
} block_t;

#define BLOCK_BUFFER_SIZE 16
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;

block_t *plan_get_current_block();
void calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor);
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next);
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next);




// ================================== Function Declarations ================================================================================
void updateDisplay(void);
void dumpPerflog(void);
void writeConfig(void);
void readConfig(void);
void kill(void);
void line_to(double, double, double, double);
void gotoXY(double, double);
void findLimits(int);


// ================================== Make Magic ================================================================================

void setup() {

// set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);         

  Serial.begin (115200);
  Serial.print("Linear Encoder Test");
  Serial.println("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_1000);

  ram.initialize();                           // Initialize free memory allocation 
  EEPROM.setMemPool(0, 2048);                 // EEPROM does not know Teensy 3.1 default EEPROM size

  if (timeStatus()!= timeSet) {
  Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  axisSetup();                                  // Initialize Axis controls

 
  pidTimer.begin(updatePID,PID_UPDATE_TIME);    // run PID update regularly

  
  pinMode(XaxisEND, INPUT); 				// set Xaxis Endstop as an input
  attachInterrupt(XaxisEND, xEndISR, FALLING); 	// Interrupt on endstop detect  

  pinMode(YaxisEND, INPUT); 				// set Yaxis Endstop as an input
  attachInterrupt(YaxisEND, yEndISR, FALLING); 	// Interrupt on endstop detect  


  delay(1000);                                  // 1 second delay to show dislay
  randomSeed(analogRead(0));                    // Used to select random SPs for testing

}

void loop() {          // NOTE:  ONLY WORKING X-AXIS for this sketch

  
    if (doOutput > 2000) { doOutput = 0; updateDisplay(); }

    get_buffer();                       // Look for and parse Serial Data.

   ram.run();                           // Monitor available memory 

}



// This routine calculates instantaneous corrections in velocity and position of X/Y axis
void updatePID(void){              

double XaxisOldPos = axis[0].ppid.pos;                // Previous X position
double YaxisOldPos = axis[1].ppid.pos;                // Previous Y position

double XaxisOldVel = axis[0].vpid.vel;                // Previous X velocity
double YaxisOldVel = axis[1].vpid.vel;                // Previous Y velocity
double Xaccel;
double Yaccel;

    axis[0].ppid.pos = xPosn.calcPosn();              // Get current Xaxis position in encoder ticks  
                                                      // velocity = abs of distance travelled per second       
    axis[0].vpid.vel =  abs(XaxisOldPos-axis[0].ppid.pos) *10000; 
                                                      // acceleration = abs change in velocity per second
    Xaccel = abs(XaxisOldVel-axis[0].vpid.vel) * 1000;
    
    xpPID.Compute();                              // Run PID assessment of current position vs target position
    axis[0].vpid.vel = axis[0].ppid.vel;          // Feed velocity PID if position need correcting                // Can we remove this step, and combine in the STRUCTURE?
    xvPID.Compute();                              // Run PID assessment of current Xaxis Velocity

    if(axis[0].vpid.vel < 0) {                    // Determine direction of travel
      digitalWrite(XaxisDir,BACKWARD);  
    } else {
      digitalWrite(XaxisDir,FORWARD);
    }      
    analogWrite(XaxisPWM,abs(axis[0].vpid.spd));      // Apply PID PWM speed to motor

  
    axis[1].ppid.pos = yPosn.calcPosn();              // Get current Yaxis position in encoder ticks  
                                                      // velocity = abs of distance travelled per second       
    axis[1].vpid.vel =  abs(YaxisOldPos-axis[1].ppid.pos) *10000; 

                                                     // acceleration = abs change in velocity per second
    Yaccel = abs(YaxisOldVel-axis[1].vpid.vel) *1000;
    
    ypPID.Compute();                              // Run PID assessment of current position vs target position
    axis[1].vpid.vel = axis[1].ppid.vel;          // Feed velocity PID if position need correcting                // Can we remove this step, and combine in the STRUCTURE?
    yvPID.Compute();                              // Run PID assessment of current Yaxis Velocity

    if(axis[1].vpid.vel < 0) {                    // Determine direction of travel
      digitalWrite(YaxisDir,BACKWARD);  
    } else {
      digitalWrite(YaxisDir,FORWARD);
    }      
    analogWrite(YaxisPWM,abs(axis[1].vpid.spd)); // Apply PID PWM speed to motor

    if (logging==true && logpos < 999) {
        logpos++;                             // Increment Position in the motion 
        perflog[logpos].Xpos = axis[0].ppid.pos;      // Current X position in encoder ticks
        perflog[logpos].Xvel = axis[0].ppid.vel;      // Current X velocity in ticks/second
        perflog[logpos].Ypos = axis[1].ppid.pos;      // Current Y position in encoder ticks
        perflog[logpos].Yvel = axis[1].ppid.vel;      // Current Y velocity in ticks/second 
    }

   motion = false;                                    // Tell line planner it can update next position 
} 


void axisSetup() {
  // Set up X axis first


  xPosn.setup(); yPosn.setup();                 // Initialize Quad Decode counters
  xPosn.start(); yPosn.start();                 // Start Quad Decode position count                        

  xpPID.SetMode(AUTOMATIC);                                   // Turn on the Xaxis Position PID loop 
  xpPID.SetSampleTime(sampleRate);                            // Sets the sample rate 
  xpPID.SetOutputLimits(0-axis[0].maxVEL,axis[0].maxVEL);     // Constrain velocity for axis

  xvPID.SetMode(AUTOMATIC);                                   // Turn on the Xaxis Position PID loop 
  xvPID.SetSampleTime(sampleRate);                            // Sets the sample rate 
  xvPID.SetOutputLimits(0-axis[0].maxPWM,axis[0].maxPWM);     // Constrain PWM for DC motor control

  pinMode(XaxisPWM, OUTPUT);                    // Assign external Pin for DC motor PWM control
  analogWriteFrequency(XaxisPWM, 46875);        // Place PWM freq outside of audible range
  pinMode(XaxisDir, OUTPUT);                    // Assign external Pin for DC motor direction control
  

  
  // Now Set up Y axis.

  ypPID.SetMode(AUTOMATIC);                                   // Turn on the Xaxis Position PID loop 
  ypPID.SetSampleTime(sampleRate);                            // Sets the sample rate 
  ypPID.SetOutputLimits(0-axis[1].maxVEL,axis[1].maxVEL);     // Constrain velocity for axis

  yvPID.SetMode(AUTOMATIC);                                   // Turn on the Xaxis Position PID loop 
  yvPID.SetSampleTime(sampleRate);                            // Sets the sample rate 
  yvPID.SetOutputLimits(0-axis[1].maxPWM,axis[1].maxPWM);     // Constrain PWM for DC motor control

  pinMode(YaxisPWM, OUTPUT);                    // Assign external Pin for DC motor PWM control
  analogWriteFrequency(YaxisPWM, 46875);        // Place PWM freq outside of audible range
  pinMode(YaxisDir, OUTPUT);                    // Assign external Pin for DC motor direction control
  
   
}

// Interrupt Service Routine for XAXIS Endstop
void xEndISR() {
	cli();
	axis[0].ppid.tpos = axis[0].ppid.pos;		// Lock current position as target in XPID
	xPosn.zeroFTM();						// Set Xaxis Encoder to Zero
	sei();
}

void yEndISR() {
	cli();
	axis[1].ppid.tpos = axis[1].ppid.pos;		// Lock current position as target in YPID
	yPosn.zeroFTM();						// Set Yaxis Encoder to Zero
	sei();
}




