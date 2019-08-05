/*
  TorchControl
*/
#ifndef TorchControl_h
#define TorchControl_h

#include "Arduino.h"

#define NUMBER_OF_READINGS 750

struct THC_Data {
    double arc_voltage;
    double set_voltage;
    //double velocity_tolorance; //Must be within X inches/min to are target velocity before ATHC starts to comp Z height
    double voltage_tolorance; //If we are whithin X volts of our target voltage, don't make Z adjustments!
    double comp_velocity; //IPM to make adjustments at.
    bool enabled;
    bool torch_on;

    int numReadings; //Number of readings to average from
    int readings[NUMBER_OF_READINGS];      // the readings from the analog input
    int readIndex;              // the index of the current reading
    double total;               // the running total
    double average;             // the average
};

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
      Return the current measured and scaled arc voltage
    */
    double get_arc_voltage();

    /*
      Return the currently set arc voltage
    */
    double get_set_voltage();

    /*
      Sets the ATHC taget voltage
    */
    void set_arc_voltage(double volts);

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

    /*
      mapfloat
    */
    double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);

    /*
      Check if two variables are within a given tolorance
    */
    bool IsInTolerance(double a, double b, double t);


    /*
      sample and average our voltage reading
    */
    void sample_voltage();

};

#endif
