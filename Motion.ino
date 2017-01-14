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


void prepare_move(void) {

  
}

void line_to(double start_x, double start_y, double end_x, double end_y) {

   /// Bresenham's Line Algorithm
  double cx = start_x;
  double cy = start_y;
 
  double dx = end_x - cx;       // Delta X
  double dy = end_y - cy;       // Delta Y
  
  if(dx<0) dx = 0-dx;
  if(dy<0) dy = 0-dy;
 
  int sx=0; int sy=0;
  if(cx < end_x) sx = 1; else sx = -1;
  if(cy < end_y) sy = 1; else sy = -1;
  int err = dx-dy;
 
 
 for(;;){                     // Loop through points in Bresenham's Line Algorithm
    if(motion == false) {     // Wait for PID routine to clear last movement
       axis[0].ppid.tpos = cx;        // Move X.
       axis[1].ppid.tpos = cy;        // Move 

       if((cx==end_x) && (cy==end_y)) return;
       
       int e2 = 2*err;
       if(e2 > (0-dy)) { err = err - dy; cx = cx + sx; }
       if(e2 < dx    ) { err = err + dx; cy = cy + sy; }

       motion = true;                  // Set flag to request Interrupt routine to move axis
    }   
 }
}
  

void gotoXY(double Xpos, double Ypos){       // Snap directly to X/Y position - Fast Non-Bresenham's Line Algorithm
            axis[0].ppid.tpos = Xpos;        // Set Target Position for X axis
            axis[1].ppid.tpos = Ypos;        // Set Target Position for X axis            
  
}

void findLimits(int whichAxis) {
  // This routine finds the position of both outer axis limits, as well as the maximum velocity and accelleration 
  // Available to the axis, and saves this to EEPROM
  
double cpos = 0;                                          // Current position in ticks;
int smooth = 0;                                           // Counter to debounce count;
int deltaPos = 0;
double timeInterval;                                     // Time interval for calculationg velocity.
double curVelocity;                                      // Current instantaneous velocity

    axis[whichAxis].ppid.pos = 0;                         // Set current position artificially to Zero.
                                                          // Set up X axis limits   
    digitalWrite(XaxisDir,BACKWARD);                      // Start by finding the Zero (left) Limit - slowly
    analogWrite(XaxisPWM,128);                            // Apply 1/2 speed to motor (so we don't cause damage when we crash into the limit)
    
    for(int x=0;x++;) {
      axis[whichAxis].ppid.pos = xPosn.calcPosn();        // Get current Xaxis position in encoder ticks  
      deltaPos = abs(cpos-axis[whichAxis].ppid.pos);      // How far have we travelled?
      cpos = axis[whichAxis].ppid.pos;                    // Set current position for next loop
      if(deltaPos<2) smooth++;                            // If no change in position, increase smooth counter
      else smooth = 0;                                    // If still travelling, keep smooth reset.
      if(smooth > 10) break;                              // If axis is stalled for more than 10 loops, assume we are at left limit.
    }
    delay(100);
    endstop[whichAxis] = axis[whichAxis].ppid.pos;        // Initialize backstop position

    digitalWrite(XaxisDir,FORWARD);                       // Start by finding the Zero (left) Limit - slowly
    analogWrite(XaxisPWM,256);                            // Apply full speed to motor to figure out maximum velocity and accelleration

    timeInterval = micros();
    for(int x=0;x++;) {

      endstop[whichAxis] = xPosn.calcPosn();              // Get current Xaxis position in encoder ticks  
      deltaPos = abs(cpos-endstop[whichAxis]);            // How far have we travelled?
      cpos = endstop[whichAxis];                          // Set current position for next loop
      
      curVelocity = deltaPos/(micros()-timeInterval);     // Current Velocity = change in position over time 
      timeInterval = micros();      

      if(curVelocity > maxAvailVel[whichAxis]) maxAvailVel[whichAxis] = curVelocity;    // Set Maximum Available Velocity
      
      if(deltaPos<2) smooth++;                            // If no change in position, increase smooth counter
      else smooth = 0;                              // If still travelling, keep smooth reset.
      if(smooth > 10) break;                        // If axis is stalled for more than 10 loops, assume we are at left limit.
    }          
    
}

void kill(void){

    axis[0].vpid.spd= 0;
    axis[0].vpid.spd= 0;  
    
}

