/*
  MotionPlanner
*/
#ifndef MotionPlanner_h
#define MotionPlanner_h

#include "Arduino.h"
#include "RingBuf.h"

#define MOVE_STACK_SIZE 50
#define FEED_VALUE_SCALE 1000.0
#define FEED_RAMP_UPDATE_INTERVAL 10 //This is in milliseconds

#define RAPID_MOVE 0
#define LINE_MOVE 1
#define SYNC_MOVE 2

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
   bool run; //This flag will be true while system is in motion and false when it is now

   /*
    This flag will be set to true when a feedhold needs to be triggered and set to false once it's been handled.
    We check if it needs to be handled during every move
    in a rapid move - we set a new deccel_marker for just ahead of our current position and set run=false once we deccel to MIN_FEED_RATE.
   */
   bool pendingFeedhold;
   bool feedholdActive;
   bool pendingSoftAbort;
};

struct Move_Data {
   Bresenham_Data Motion;
   XYZ_Long target;
   /*
    Use distance markers to deterine when acceleration/decceleration need to begin
   */
   double accel_marker;
   double deccel_marker;
   double feedhold_marker;
   /*
    Keep track of the entry and exit velocitys. They should be set to the minimum feedrate, and markers should be caclulated to reflect this.
    When the planner re-calculates for continnous motion, the entry and exit velocities need to be updates and the markers do as well!
   */
   double entry_velocity; //units/sec
   double exit_velocity; //units/sec

   uint8_t move_type;
   void (*sync_callback)();
   bool waiting_for_sync;
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
      Stop the motion interupt
    */
    void stop();

    /*
      Feed hold the current motion
    */
    void feedhold();

    /*
      Sets the run flag to true
    */
    void run();

    /*
      Clear stack and halt motion
    */
    void abort();

    /*
      Feedhold then abort once the feedhold is complete
    */
    void soft_abort();

    /*
      Pushes a new target to the target buffer and updates feedramps for constant motion based on the other moves on the stack
      returns false if the target buffer is full.

      target - XYZ_Double set to a position in scaled units and f is feedrate in units/min
    */
    bool push_target(XYZ_Double target, uint8_t move_type);

    /*
      Pushes a new sync move to the stack

      callback - This is the call that needs to be ran once the head of the stack gets to this item. Needs to be non-blocking so main loop still cycles.
      Motion will resume once motion.sync_finished is called!
    */
    bool push_sync(void (*callback)());

    /*
      Notifies that it's time to move onto next move
    */
   void sync_finished();

    /*
      Returns a XYZ_Double with current machine coordinant positions
    */
    XYZ_Double get_current_position();

    /*
      Returns a XYZ_Double with the target position of the last move on the stack in scaled units
    */
    XYZ_Double get_last_moves_target();

    /*
      Returns a XYZ_Long with the target position of the last move on the stack in step units
    */
    XYZ_Long get_last_moves_target_steps();

    /*
      Returns a XYZ_Double with current axis velocities
      f will be cartesion velocity
    */
    XYZ_Double get_current_velocity();

    /*
      Used for debugging purposes. Dumps all the data for current move
    */
    void dump_current_move_to_serial();

    /*
      Return whether or not we are in motion
    */
    bool is_in_motion();

    /*
      Invert axis step direction
    */
    void invert_joint_dir(int axis, int value);

    /*
      Set axis scale
    */
    void set_axis_scale(int axis, double value);

    /*
      This is called by an interupt timer. Handles all syncronized motion. Not meant to be user callable but needs to be public so ISR can reference it
    */
    void motion_tick();

    /*
      This is called from main loop
    */
    void tick();



  private:
    volatile unsigned long _Feed_Sample_Timestamp;
    volatile unsigned long _Feedrate_Timestamp;
    volatile unsigned long _Feedrate_delay;

    volatile bool _Invert_X_Dir;
    volatile bool _Invert_Y1_Dir;
    volatile bool _Invert_Y2_Dir;

    //Contants that need to be set by constructor initially and they need to have public setters
    XYZ_Double _Step_Scale;
    XYZ_Double _Feed_Jerk;
    XYZ_Double _Feed_Accel;

    volatile XYZ_Long CurrentPosition; //Holds the current position in step units
    volatile XYZ_Long TargetPosition; //Holds the target position in step units
    XYZ_Double CurrentVelocity; //Stores velocity for each axis in inches/sec
    double percentage_into_move; //This store the percentage complete through the current move, needs to be global in order to insert feedholds into ramp_map

    volatile Move_Data CurrentMove; //Holds the current move that has been popped of the top of the move stack
    volatile Bresenham_Data Motion;

    /*
      Sets the new target position. Reads CurrentMove to get X,Y target data and sets the breseham variables in Move

      Obsolete: this used to be called from the timer ISR after a move finishes, but causes a short stop between moves at feedrates which have a delay shorter than
      the amount of time it takes to caclulate the new bresenham data. Now we pre-cacluate this data and it's stored in the Move_Data
    */
    void motion_set_target();

    /*
      Pre-Calculates Motion data, called by push_target

      cur_pos - XYZ_Long of the "currrent_pos" in the move stack
      tar_pos - XYZ_Long of the new "target_pos" 
    */
    Bresenham_Data motion_calculate_target(XYZ_Long cur_pos, XYZ_Long tar_pos);

    /*
      Calculates the feed delay required to throttle the bresehm loop to the desired feedrate
      Called by the motion interupt after calculating the move precentage.

      feed - units are units/sec
    */
    void motion_set_feedrate(double feed);

    /*
      Updates the previously calculated "exact stop" markers and updates them for continous motion
        - this should be called everytime a new target is pushed to the stack
    */
    void motion_plan_moves_for_continuous_motion();
    void motion_plan_moves_for_continuous_motion_junk();

    /*
      Return a polar angle between two cartesion points
    */
    double motion_get_vector_angle(XYZ_Double p1, XYZ_Double p2);

    /*
      Return a relative angle between two vectors
    */
    double motion_get_relative_angle_between_vectors(XYZ_Double l1p1, XYZ_Double l1p2, XYZ_Double l2p1, XYZ_Double l2p2);

    /*
      Return input radians in degrees
    */
    double to_degrees(double radians);

    /*
      Return input degrees in radians
    */
    double to_radians(double degrees);

    /*
      Return the distance required to accelerate to a target velocity. Assumes that motion starts at MIN_FEED_RATE

      accel_rate - Acceleration rate in units/sec
      target_velocity - Target velocity in units/sec
    */
    double motion_calculate_accel_marker(double accel_rate, double target_velocity);

    /*
      Return the feedrate at a specific distance into move

      accel_rate - Acceleration rate in units/sec
      distance_into_move - how far have we traveled into the move in scaled units. This should be the distance the dominant axis has traveled
    */
    double motion_calculate_feed_from_distance(double accel_rate, double distance_into_move);

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
