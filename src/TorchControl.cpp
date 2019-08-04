/*
  TorchControl
*/
#include "Arduino.h"
#include "TorchControl.h"
#include "RingBuf.h"
#include "Gcodes.h"
#include "Machine.h"
#include <IntervalTimer.h>

IntervalTimer move_timer;
unsigned long torch_fired_timestamp = 0; //Keep track of when the torch turns on for ATHC
bool (*ConditionCallback)(); //This callback will stop the move if it returns true
void (*ConditionMetCallback)(); //This callback gets called once the condition callback returns true

void move_timer_tick()
{
  torch.move_tick();
}
TorchControl::TorchControl()
{
  _Step_Scale = 2540;
  _Feedrate_Timestamp = 0;
  _Feedrate_delay = 0;
  CurrentPosition = 0;
  StepsToGo = 0;
  StepDir = 0;
  run = false;
}
void TorchControl::init()
{
  pinMode(Z_STEP_PIN, OUTPUT); //Z Step
  pinMode(Z_DIR_PIN, OUTPUT); //Z Dir
  pinMode(ARC_START_PIN, OUTPUT); //Arc Start
  pinMode(Z_PROBE_PIN, INPUT); //Z Probe

  if (move_timer.begin(move_timer_tick, 10))
  {
    printf(Serial, "move_timer Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "move_timer Timer init: FAIL!\n");
  }
  _Feedrate_delay = cycle_frequency_from_feedrate(MIN_FEED_RATE);
}
void TorchControl::cancel()
{
  run = false;
}
void TorchControl::move_z_incremental(double distance, double feedrate, bool (*condition)(), void (*met)())
{
  run = false; //If we are already moving, stop the move!
  ConditionCallback = condition;
  ConditionMetCallback = met;
  _Feedrate_delay = cycle_frequency_from_feedrate(feedrate);
  StepsToGo = fabs(distance) * _Step_Scale;
  if (distance > 0)
  {
    StepDir = 1;
  }
  else
  {
    StepDir = -1;
  }
  run = true;
}
void TorchControl::tick()
{

}
unsigned long TorchControl::cycle_frequency_from_feedrate(double feedrate)
{
  return ((1000 * 1000) / (_Step_Scale)) / feedrate;
}
void TorchControl::move_tick()
{
  if (run == true)
  {
    if (ConditionCallback != NULL)
    {
      if (ConditionCallback() == true)
      {
        ConditionCallback = NULL; //make sure the callback is cleared so it only runs once
        //The callback is responsable for stopping motion
        if (ConditionMetCallback != NULL) ConditionMetCallback();
      }
      return;
    }
    if (micros() > _Feedrate_Timestamp + _Feedrate_delay)
    {
      if (StepsToGo > 0)
      {
        step_z(StepDir);
      }
      else
      {
        run = false;
        StepDir = 0;
        if (ConditionMetCallback != NULL) ConditionMetCallback();
      }
      _Feedrate_Timestamp = micros();
    }
  }
}
void TorchControl::step_z(int dir)
{
  if (dir == 0) return; //Something was wrong if this happens. Just make sure the pins arn't toggles to are counters are accurate
  CurrentPosition += dir;
  StepsToGo--;
  if (dir > 0)
  {
    digitalWrite(Z_DIR_PIN, HIGH);
  }
  else
  {
    digitalWrite(Z_DIR_PIN, LOW);
  }
  delayMicroseconds(20); //Delay for direction change
  digitalWrite(Z_STEP_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(Z_STEP_PIN, HIGH);
  //printf(Serial, "Z position: %ld\n", CurrentPosition);
}
void TorchControl::fire_torch()
{
  torch_fired_timestamp = millis();
  digitalWrite(28, HIGH);
}
void TorchControl::extinguish_torch()
{
  digitalWrite(28, LOW);
}
