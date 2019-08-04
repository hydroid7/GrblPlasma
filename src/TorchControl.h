/*
  TorchControl
*/
#ifndef TorchControl_h
#define TorchControl_h

#include "Arduino.h"

class TorchControl
{
  public:
    TorchControl();

    /*
      Sets up hardware io for the Z axis and torch height control
    */
    void init();

    /*
      Engage the contacts for Arc Start and keep track of when it was started
    */
    void fire_torch();

    /*
      Turn of the arc!
    */
    void extinguish_torch();

    /*
      Move the z axis incrementally, this is a non-blocking call.

      distance - distance to travel, sign is direction
      feedrate - rate to travel at in units/sec
      callback - A function that gets called every cycle tick, when it returns true the motion is halted
          ^ ------ NULL can be passed if there's no reason to stop the move before it finishes
    */
    void move_z_incremental(double distance, double feedrate, bool (*condition)(), void (*met)());

    /*
      Cancel Move
    */
    void cancel();

    /*
      Tick torch control, needs to be called every loop cycle.
    */
    void tick();

    /*
      Tick torch control, called by the motion timer interupt
    */
    void move_tick();

    /*
      Step the z axis

      dir - +1 or -1
    */
    void step_z(int dir);

  private:
    double _Step_Scale;
    unsigned long _Feedrate_Timestamp;
    unsigned long _Feedrate_delay;
    long CurrentPosition;
    long StepsToGo;
    int StepDir;
    bool run;

    /*
      Calculate feed_delay given feedrate in units/sec
    */
    unsigned long cycle_frequency_from_feedrate(double feedrate);

};

#endif
