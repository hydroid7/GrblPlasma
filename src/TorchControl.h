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
      Move the z axis incrementally, this is a non-blocking call.

      distance - distance to travel, sign is direction
    */
    void move_z_incremental(double distance, double feedrate);

    /*
      Block until Z move completets, this will call void loop internally so that it doesn't block everything
    */
    void wait_for_move_to_finish();

    /*
      Tick torch control, needs to be called every loop cycle
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
