/**
 * planner.cpp - Buffer movement commands and manage the acceleration profile plan
 * Part of Grbl
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 */

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define BIT(b) (1UL << (b))

#define EXTRUDERS 1

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)
#define dropsegments 2                            // Minimum number of allowed segments in ring

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

//  millis_t minsegmenttime;
float max_feedrate[NUM_AXIS]; // Max speeds in mm per minute
#define DEFAULT_MAX_FEEDRATE          {130, 130, 1.7, 20}    // (mm/sec)

float axis_steps_per_unit[NUM_AXIS];
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {35.556, 35.556, 800.00, 368.421}   // default steps per unit 
#define DEFAULT_AXIS_STEPS_PER_UNIT   {100, 100, 100, 100}                  // default steps per unit 



unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
float max_xy_jerk;          // The largest speed change requiring no acceleration
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];
static float feedrate = 1500.0, next_feedrate, saved_feedrate;


//===========================================================================
//============ semi-private variables, used in inline functions =============
//===========================================================================

extern block_t block_buffer[];            // A ring buffer for motion instructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//============================ private variables ============================
//===========================================================================

// The current position of the tool in absolute steps
long position[NUM_AXIS];               // Rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_AXIS]; // Speed of previous path line segment
static float previous_nominal_speed;   // Nominal speed of previous path line segment

uint8_t g_uc_extruder_last_move[EXTRUDERS] = { 0 };


//===========================================================================
//================================ functions ================================
//===========================================================================

// Get the next / previous index of the next block in the ring buffer
// NOTE: Using & here (not %) because BLOCK_BUFFER_SIZE is always a power of 2
int8_t next_block_index(int8_t block_index) { return BLOCK_MOD(block_index + 1); }
int8_t prev_block_index(int8_t block_index) { return BLOCK_MOD(block_index - 1); }


int square(int b)
{
    return b*b;
}

char active_extruder = 0;
int fanSpeed=0;

int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2



// Initialize the motion plan subsystem
void plan_init();

void check_axes_activity();

float volumetric_multiplier[EXTRUDERS] = {1.0};
int extruder_multiplier[EXTRUDERS] = {100};
 

// Get the number of buffered moves
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;
uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

void plan_set_position(const float& x, const float& y, const float& z, const float& e);

void plan_set_e_position(const float& e);

//===========================================================================
//============================= public variables ============================
//===========================================================================

//extern millis_t minsegmenttime;

extern float axis_steps_per_unit[];
extern unsigned long max_acceleration_units_per_sq_second[]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
extern float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
extern float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
extern float max_xy_jerk;          // The largest speed change requiring no acceleration
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[];
extern long previous_millis_cmd;

extern bool relative_mode;


static float offset[3] = {0.0, 0.0, 0.0};

extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail;







// Initialize the motion plan subsystem
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in
// millimaters. Feed rate specifies the speed of the motion.
void plan_buffer_line(const float &, const float &, const float &, const float &, float, const uint8_t &);

// Set position. Used for G92 instructions.
void plan_set_position(const float &x, const float &y, const float &z, const float &e);
void plan_set_e_position(const float &e);



void check_axes_activity();
uint8_t movesplanned(); //return the nr of buffered moves


// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);
  }
}

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) {
    return(NULL);
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

// Gets the current block. Returns NULL if buffer empty
bool blocks_queued()
{
  if (block_buffer_head == block_buffer_tail) {
    return false;
  }
  else
    return true;
}


void reset_acceleration_rates();




// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
    if (acceleration == 0) return 0; // acceleration was 0, set acceleration distance to 0
    return (target_rate * target_rate - initial_rate * initial_rate) / (acceleration * 2);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
{
    if (acceleration == 0) return 0; // acceleration was 0, set intersection distance to 0
    return (acceleration * 2 * distance - initial_rate * initial_rate + final_rate * final_rate) / (acceleration * 4);
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor)
{
    unsigned long initial_rate = ceil(block->nominal_rate * entry_factor); // (step/min)
    unsigned long final_rate = ceil(block->nominal_rate * exit_factor); // (step/min)

    long acceleration = block->acceleration_st;
    int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
    int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

    // Calculate the size of Plateau of Nominal Rate.
    int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
        accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
        accelerate_steps =((uint32_t)accelerate_steps < block->step_event_count) ? (uint32_t)accelerate_steps : block->step_event_count;//(We can cast here to unsigned, because the above line ensures that we are above zero)
        plateau_steps = 0;
    }

    // block->accelerate_until = accelerate_steps;
    // block->decelerate_after = accelerate_steps+plateau_steps;
        cli();                                                              // Disable Interrupts temporarily
    if (!block->busy) { // Don't update variables if block is busy.
        block->accelerate_until = accelerate_steps;
        block->decelerate_after = accelerate_steps + plateau_steps;
        block->initial_rate = initial_rate;
        block->final_rate = final_rate;
    }
    sei();                                                                  // Reenable Interrupts.
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next)
{
    if(!current) {
        return;
    }

    if (next) {
        // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
        // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
        // check for maximum allowable speed reductions to ensure maximum possible planned speed.
        if (current->entry_speed != current->max_entry_speed) {

            // If nominal length true, max junction speed is guaranteed to be reached. Only compute
            // for max allowable speed if block is decelerating and nominal length is false.
            if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
                current->entry_speed = min( current->max_entry_speed,
                                            max_allowable_speed(float(-current->acceleration),next->entry_speed,current->millimeters));
            } else {
                current->entry_speed = current->max_entry_speed;
            }
            current->recalculate_flag = true;

        }
    } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass()
{
    uint8_t block_index = block_buffer_head;

    //Make a local copy of block_buffer_tail, because the interrupt can alter it
    cli();
    unsigned char tail = block_buffer_tail;
    sei();

    if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
        block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
        block_t *block[3] = {
            NULL, NULL, NULL
        };
        while(block_index != tail) {
            block_index = prev_block_index(block_index);
            block[2]= block[1];
            block[1]= block[0];
            block[0] = &block_buffer[block_index];
            planner_reverse_pass_kernel(block[0], block[1], block[2]);
        }
    }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next)
{
    if(!previous) {
        return;
    }

    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag) {
        if (previous->entry_speed < current->entry_speed) {
            double entry_speed = min( current->entry_speed,
                                      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

            // Check for junction speed change
            if (current->entry_speed != entry_speed) {
                current->entry_speed = entry_speed;
                current->recalculate_flag = true;
            }
        }
    }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass()
{
    uint8_t block_index = block_buffer_tail;
    block_t *block[3] = {
        NULL, NULL, NULL
    };

    while(block_index != block_buffer_head) {
        block[0] = block[1];
        block[1] = block[2];
        block[2] = &block_buffer[block_index];
        planner_forward_pass_kernel(block[0],block[1],block[2]);
        block_index = next_block_index(block_index);
    }
    planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids()
{
    int8_t block_index = block_buffer_tail;
    block_t *current;
    block_t *next = NULL;

    while(block_index != block_buffer_head) {
        current = next;
        next = &block_buffer[block_index];
        if (current) {
            // Recalculate if current block entry or exit junction speed has changed.
            if (current->recalculate_flag || next->recalculate_flag) {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
                                              next->entry_speed/current->nominal_speed);
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            }
        }
        block_index = next_block_index( block_index );
    }
    // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
    if(next != NULL) {
        calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
                                      MINIMUM_PLANNER_SPEED/next->nominal_speed);
        next->recalculate_flag = false;
    }
}// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster acceleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate()
{
    planner_reverse_pass();
    planner_forward_pass();
    planner_recalculate_trapezoids();
}

void plan_init()
{
    block_buffer_head = block_buffer_tail = 0;
    memset(position, 0, sizeof(position)); // clear position
    for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
    previous_nominal_speed = 0.0;
    

  
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    
    for (short i=0;i<4;i++)
    {
        axis_steps_per_unit[i]=tmp1[i];
        max_feedrate[i]=tmp2[i];
    }


}



void check_axes_activity()
{
    unsigned char axis_active[NUM_AXIS] = { 0 },
                                          tail_fan_speed = fanSpeed;

    block_t* block;

    if (blocks_queued()) {
        uint8_t block_index = block_buffer_tail;
        tail_fan_speed = block_buffer[block_index].fan_speed;

        while (block_index != block_buffer_head) {
            block = &block_buffer[block_index];
            for (int i = 0; i < NUM_AXIS; i++) if (block->steps[i]) axis_active[i]++;
            block_index = next_block_index(block_index);
        }
    }

}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps[X_AXIS], _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.

void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t& extruder)
{
    // Calculate the buffer head after we push this byte
    int next_buffer_head = next_block_index(block_buffer_head);

    // If the buffer is full: good! That means we are well ahead of the robot.
    // Rest here until there is room in the buffer.
    while (block_buffer_tail == next_buffer_head) { };                // *************************************** Blocking ????  ***********************************************


    // The target position of the tool in absolute steps
    // Calculate target position in absolute steps
    //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
    long target[NUM_AXIS];
    target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
    target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
    target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
    target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

    float dx = target[X_AXIS] - position[X_AXIS],
          dy = target[Y_AXIS] - position[Y_AXIS],
          dz = target[Z_AXIS] - position[Z_AXIS];

    float de = target[E_AXIS] - position[E_AXIS];


    // Prepare to set up new block
    block_t* block = &block_buffer[block_buffer_head];

    // Mark block as not busy (Not executed by the stepper interrupt)
    block->busy = false;

    // Number of steps for each axis
    // default non-h-bot planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[Z_AXIS] = labs(dz);

    block->steps[E_AXIS] = labs(de);
    block->steps[E_AXIS] *= volumetric_multiplier[extruder];
    block->steps[E_AXIS] *= extruder_multiplier[extruder];
    block->steps[E_AXIS] /= 100;
    block->step_event_count = max(block->steps[X_AXIS], max(block->steps[Y_AXIS], max(block->steps[Z_AXIS], block->steps[E_AXIS])));
//    block->step_event_count = max(block->steps[X_AXIS], block->steps[Y_AXIS]);

printf("Currently in plan_buffer_line\r\n");
printf("Block X/Y/Z/E Event-Count %ld %ld %ld %ld %ld\r\n", block->steps[X_AXIS],target[Y_AXIS],block->steps[Z_AXIS],block->steps[E_AXIS],block->step_event_count);
    // Bail if this is a zero-length block
    if (block->step_event_count <= dropsegments) return;

    block->fan_speed = fanSpeed;


    // Compute direction bits for this block
    uint8_t db = 0;

    if (dx < 0) db |= BIT(X_AXIS);
    if (dy < 0) db |= BIT(Y_AXIS);
    if (dz < 0) db |= BIT(Z_AXIS);

    if (de < 0) db |= BIT(E_AXIS);
    block->direction_bits = db;

    block->active_extruder = extruder;

    //enable active axes

//    if (block->steps[X_AXIS]) enable_x();
//    if (block->steps[Y_AXIS]) enable_y();

    // Enable extruder(s)
    if (block->steps[E_AXIS]) {
//        enable_e0();
        g_uc_extruder_last_move[0] = BLOCK_BUFFER_SIZE * 2;
    }

    if (block->steps[E_AXIS]) {
        if(feed_rate < minimumfeedrate) feed_rate = minimumfeedrate;
        else if(feed_rate < mintravelfeedrate) feed_rate = mintravelfeedrate;
    }    
printf("feed_rate = %f\r\n",feed_rate);        

    /**
     * This part of the code calculates the total length of the movement.
     * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
     * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
     * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
     * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
     * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
     */

    float delta_mm[4];
    delta_mm[X_AXIS] = dx / axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = dy / axis_steps_per_unit[Y_AXIS];
    delta_mm[Z_AXIS] = dz / axis_steps_per_unit[Z_AXIS];

    delta_mm[E_AXIS] = (de / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[extruder] * extruder_multiplier[extruder] / 100.0;

    if (block->steps[X_AXIS] <= dropsegments && block->steps[Y_AXIS] <= dropsegments && block->steps[Z_AXIS] <= dropsegments) {
        block->millimeters = fabs(delta_mm[E_AXIS]);
    } else {
        block->millimeters = sqrt(float (square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS])));
    }
    float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides


    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
    float inverse_second = feed_rate * inverse_millimeters;

printf("block->millimeters = %f, inverse_millimeters = %f, inverse_second= %f  \r\n", block->millimeters, inverse_millimeters,inverse_second);

    int moves_queued = movesplanned();

    block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
    block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0
printf("block->nominal_speed = %f, block->nominal_rate= %ld, moves_queued = l%d  \r\n", block->nominal_speed, block->nominal_rate, moves_queued);

    // Calculate and limit speed in mm/sec for each axis
    float current_speed[NUM_AXIS];
    float speed_factor = 1.0; //factor <=1 do decrease speed
    for (int i = 0; i < NUM_AXIS; i++) {
        current_speed[i] = delta_mm[i] * inverse_second;
        float cs = fabs(current_speed[i]), mf = max_feedrate[i];
        if (cs > mf) speed_factor = min(speed_factor, mf / cs);
    }


    // Correct the speed
    if (speed_factor < 1.0) {
        for (unsigned char i = 0; i < NUM_AXIS; i++) current_speed[i] *= speed_factor;
        block->nominal_speed *= speed_factor;
        block->nominal_rate *= speed_factor;
    }
printf("After Speed Correction:\r\n");
printf("block->nominal_speed = %f, block->nominal_rate= %ld, moves_queued = %ld  \r\n", block->nominal_speed, block->nominal_rate, moves_queued);

    // Compute and limit the acceleration rate for the trapezoid generator.
    float steps_per_mm = block->step_event_count / block->millimeters;
    long bsx = block->steps[X_AXIS], bsy = block->steps[Y_AXIS], bsz = block->steps[Z_AXIS], bse = block->steps[E_AXIS];
    if (bsx == 0 && bsy == 0 && bsz == 0) {
        block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    } else if (bse == 0) {
        block->acceleration_st = ceil(travel_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    } else {
        block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    }
    // Limit acceleration per axis
    unsigned long acc_st = block->acceleration_st,
                  xsteps = axis_steps_per_sqr_second[X_AXIS],
                  ysteps = axis_steps_per_sqr_second[Y_AXIS],
                  zsteps = axis_steps_per_sqr_second[Z_AXIS],
                  esteps = axis_steps_per_sqr_second[E_AXIS];
    if ((float)acc_st * bsx / block->step_event_count > xsteps) acc_st = xsteps;
    if ((float)acc_st * bsy / block->step_event_count > ysteps) acc_st = ysteps;
    if ((float)acc_st * bsz / block->step_event_count > zsteps) acc_st = zsteps;
    if ((float)acc_st * bse / block->step_event_count > esteps) acc_st = esteps;

    block->acceleration_st = acc_st;
    block->acceleration = acc_st / steps_per_mm;
    block->acceleration_rate = (long)(acc_st * 16777216.0 / (F_CPU / 8.0));

#if 0  // Use old jerk for now
    // Compute path unit vector
    double unit_vec[3];

    unit_vec[X_AXIS] = delta_mm[X_AXIS] * inverse_millimeters;
    unit_vec[Y_AXIS] = delta_mm[Y_AXIS] * inverse_millimeters;
    unit_vec[Z_AXIS] = delta_mm[Z_AXIS] * inverse_millimeters;

    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
        // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
        // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
        double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                           - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                           - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
        // Skip and use default max junction speed for 0 degree acute junction.
        if (cos_theta < 0.95) {
            vmax_junction = min(previous_nominal_speed, block->nominal_speed);
            // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
            if (cos_theta > -0.95) {
                // Compute maximum junction velocity based on maximum acceleration and junction deviation
                double sin_theta_d2 = sqrt(0.5 * (1.0 - cos_theta)); // Trig half angle identity. Always positive.
                vmax_junction = min(vmax_junction,
                                    sqrt(block->acceleration * junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2)));
            }
        }
    }
#endif

    // Start with a safe speed
    float vmax_junction = max_xy_jerk / 2;
    float vmax_junction_factor = 1.0;
    float mz2 = max_z_jerk / 2, me2 = max_e_jerk / 2;
    float csz = current_speed[Z_AXIS], cse = current_speed[E_AXIS];
    if (fabs(csz) > mz2) vmax_junction = min(vmax_junction, mz2);
    if (fabs(cse) > me2) vmax_junction = min(vmax_junction, me2);
    vmax_junction = min(vmax_junction, block->nominal_speed);
    float safe_speed = vmax_junction;

    if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
        float dx = current_speed[X_AXIS] - previous_speed[X_AXIS],
              dy = current_speed[Y_AXIS] - previous_speed[Y_AXIS],
              dz = fabs(csz - previous_speed[Z_AXIS]),
              de = fabs(cse - previous_speed[E_AXIS]),
              jerk = sqrt(dx * dx + dy * dy);

        //    if ((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
        vmax_junction = block->nominal_speed;
        //    }
        if (jerk > max_xy_jerk) vmax_junction_factor = max_xy_jerk / jerk;
        if (dz > max_z_jerk) vmax_junction_factor = min(vmax_junction_factor, max_z_jerk / dz);
        if (de > max_e_jerk) vmax_junction_factor = min(vmax_junction_factor, max_e_jerk / de);

        vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
    }
    block->max_entry_speed = vmax_junction;

    // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
    double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
//  block->entry_speed = min(vmax_junction, v_allowable);
    block->entry_speed = (vmax_junction < v_allowable) ? vmax_junction : v_allowable;

    // Initialize planner efficiency flags
    // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
    // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
    // the current block and next block junction speeds are guaranteed to always be at their maximum
    // junction speeds in deceleration and acceleration, respectively. This is due to how the current
    // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
    // the reverse and forward planners, the corresponding block junction speed will always be at the
    // the maximum junction speed and may always be ignored for any speed reduction checks.
    block->nominal_length_flag = (block->nominal_speed <= v_allowable);
    block->recalculate_flag = true; // Always calculate trapezoid for new block

    // Update previous path unit_vector and nominal speed
    for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = current_speed[i];
    previous_nominal_speed = block->nominal_speed;



    calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

    // Move buffer head
    block_buffer_head = next_buffer_head;

    // Update position
    for (int i = 0; i < NUM_AXIS; i++) position[i] = target[i];

    planner_recalculate();

 //   st_wake_up();

} // plan_buffer_line()


void plan_set_position(const float& x, const float& y, const float& z, const float& e)
{

    float nx = position[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]),
                                  ny = position[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]),
                                          nz = position[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]),
                                                  ne = position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
    st_set_position(nx, ny, nz, ne);
    previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.

    for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
}

void plan_set_e_position(const float& e)
{
    position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
    st_set_e_position(position[E_AXIS]);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates()
{
    for (int i = 0; i < NUM_AXIS; i++)
        axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
}

void prepare_move()
{

    Serial.printf("Preparing Move: X %f, Y %f\r\n",destination[X_AXIS],destination[Y_AXIS]);        //  ******************************** DEBUG  **********************
//       clamp_to_software_endstops(destination);                                                   //  ******************************** Needed?? **********************

    previous_millis_cmd = millis();

    // Do not use feedmultiply for E or Z only moves
    if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, uint8_t(active_extruder));
    } else {
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, uint8_t(active_extruder));
    }
    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    for(int8_t i=0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];       // Current position is actually "planner current position" going into next block, not accounting for where steppers are...........
    }
}

void prepare_arc_move(char isclockwise)
{
    float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

    // Trace the arc
    mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    for(int8_t i=0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];
    }
    previous_millis_cmd = millis();
}

// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1,
            uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[axis_0] + offset[axis_0];
    float center_axis1 = position[axis_1] + offset[axis_1];
    float linear_travel = target[axis_linear] - position[axis_linear];
    float extruder_travel = target[E_AXIS] - position[E_AXIS];
    float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
    float r_axis1 = -offset[axis_1];
    float rt_axis0 = target[axis_0] - center_axis0;
    float rt_axis1 = target[axis_1] - center_axis1;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
    if (angular_travel < 0) {
        angular_travel += 2*M_PI;
    }
    if (isclockwise) {
        angular_travel -= 2*M_PI;
    }

    float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001) {
        return;
    }
    uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
    if(segments == 0) segments = 1;

    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = linear_travel/segments;
    float extruder_per_segment = extruder_travel/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. This requires only two cos() and sin() computations to form the rotation
       matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       all double numbers are single precision on the Arduino. (True double precision will not have
       round off issues for CNC applications.) Single precision error can accumulate to be greater than
       tool precision in some cases. Therefore, arc path correction is implemented.

       Small angle approximation may be used to reduce computation overhead further. This approximation
       holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[4];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[axis_linear] = position[axis_linear];

    // Initialize the extruder axis
    arc_target[E_AXIS] = position[E_AXIS];

    for (i = 1; i<segments; i++) { // Increment (segments-1)

        if (count < N_ARC_CORRECTION) {
            // Apply vector rotation matrix
            r_axisi = r_axis0*sin_T + r_axis1*cos_T;
            r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cos(i*theta_per_segment);
            sin_Ti = sin(i*theta_per_segment);
            r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
            r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[axis_0] = center_axis0 + r_axis0;
        arc_target[axis_1] = center_axis1 + r_axis1;
        arc_target[axis_linear] += linear_per_segment;
        arc_target[E_AXIS] += extruder_per_segment;

//    clamp_to_software_endstops(arc_target);                                  *********************************************  DEBUG  ************************
        plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, uint8_t(extruder));

    }
    // Ensure last segment arrives at target location.
    plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feed_rate, uint8_t(extruder));

    //   plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
}


void get_coordinates()
{
    bool seen[4]= {false,false,false,false};
    for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
            destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
            seen[i]=true;
            Serial.printf("Destination %u = %f  \r\n", i, destination[i]);             // ******************************************* DEBUG  ***************************8
        } else destination[i] = current_position[i]; //Are these else lines really needed?
    }
    if(code_seen('F')) {
        next_feedrate = code_value();
        if(next_feedrate > 0.0) feedrate = next_feedrate;
    }
}

void get_arc_coordinates()
{
    get_coordinates();

    if(code_seen('I')) {
        offset[0] = code_value();
    } else {
        offset[0] = 0.0;
    }
    if(code_seen('J')) {
        offset[1] = code_value();
    } else {
        offset[1] = 0.0;
    }
}



