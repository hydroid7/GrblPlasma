/*
  MotionPlanner
*/
#ifndef MotionPlanner_h
#define MotionPlanner_h

#include "Arduino.h"
#include "RingBuf.h"

#define RAMP_MAP_SIZE 1000 //This gives us a resolution of 1000 feedrate changes on the ramp map
#define MOVE_STACK_SIZE 50
#define FEED_RAMP_SCALE 1000
#define FEED_RAMP_UPDATE_INTERVAL 25 //This is in milliseconds

struct XYZ_Long {
   long x;
   long y;
   long z;
   long f;
};
struct XYZ_Double {
   double x;
   double y;
   double z;
   double f;
};
struct Bresenham_Data {
   long dx;
   long dy;
   int sx;
   int sy;
   int err;
   long x_stg;
   long y_stg;
   bool run;
};

struct Move_Data {
   XYZ_Long target;
   /*
    Functions that need use the ramp_map need to remember that the feedramp values are scaled by FEED_RAMP_SCALE to avoid storing floats
    unsigned int can store a value betwee 0 and 65,535 and store a feedrate from 0-3,932.1 units/sec with a FEED_RAMP_SCALE of 1000
    Storing 1000 float would be 32000 bits
    Storing 1000 int is 16000 bits, half the size per element and can still store a feedrate over 1000
  */
   unsigned int ramp_map[RAMP_MAP_SIZE];
};
extern RingBuf *MoveStack;

class MotionPlanner
{
  public:
    MotionPlanner();

    /*
      Sets up the motion timer interupt
    */
    void init();

    /*
      Pushes a new target to the target buffer and updates feedramps for constant motion based on the other moves on the stack
      returns false if the target buffer is full.

      target - XYZ_Double set to a position in scaled units and f is feedrate in units/min
    */
    bool push_target(XYZ_Double target);

    /*
      Returns a XYZ_Double with current machine coordinant positions
    */
    XYZ_Double get_current_position();

    /*
      Returns a XYZ_Double with current axis velocities
      f will be cartesion velocity
    */
    XYZ_Double get_current_velocity();

    /*
      This is called by an interupt timer. Handles all syncronized motion
    */
    void motion_tick();



  private:
    volatile unsigned long _Feed_Sample_Timestamp;
    volatile unsigned long _Feedrate_Timestamp;
    volatile unsigned long _Feedrate_delay;

    //Contants that need to be set by constructor initially and they need to have public setters
    XYZ_Double _Step_Scale;
    XYZ_Double _Feed_Jerk;
    XYZ_Double _Feed_Accel;

    XYZ_Long CurrentPosition; //Holds the current position in step units
    XYZ_Long TargetPosition; //Holds the target position in step units
    XYZ_Double CurrentVelocity; //Stores velocity for each axis in inches/sec

    Move_Data CurrentMove; //Holds the current move that has been popped of the top of the move stack
    Bresenham_Data Motion;

    /*
      Sets the new target position. Reads CurrentMove to get X,Y target data and sets the breseham variables in Move
    */
    void motion_set_target();

    /*
      Calculates the feed delay required to throttle the bresehm loop to the desired feedrate
      Called by the motion interupt after calculating the move precentage.

      feed - units are units/sec
    */
    void motion_set_feedrate(double feed);

    /*
      Updates the previously calculated "exact stop" feed ramps and updates them for continous motion
      Uses a mapped differental to determine the exit_velocity based on the vector angle change of the moves,
      so slight angle changes only warrent a small change in velocity and large angle changes warrent a large
      deccel before the change happens

        - this should be called everytime a new target is pushed to the stack
    */
    void motion_plan_moves_on_stack();

    /*
      This modifies the entry ramp of a move to reflect a new entry velocity

        - this should only be called by motion_plan_moves_on_stack()
      move_index - is the index position on the target stack that contains the ramp that needs to be updated
      entry_velocity  - the new entry velocity in units/sec
    */
    void motion_recalculate_ramp_map_for_entry(int move_index, double entry_velocity);

    /*
      This modifies the exit ramp of a move to reflect a new exit velocity

        - this should only be called by motion_plan_moves_on_stack()
    move_index - is the index position on the target stack that contains the ramp that needs to be updated
    exit_velocity  - the new exit velocity in units/sec
    */
    void motion_recalculate_ramp_map_for_exit(int move_index, double exit_velocity);

    /*
      Code to step_x
    */
    void motion_step_x(int dir);

    /*
      Code to step_y
    */
    void motion_step_y(int dir);



};

#endif
