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
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
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
void TorchControl::wait_for_move_to_finish()
{

}
void TorchControl::move_z_incremental(double distance, double feedrate)
{
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
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(5, LOW);
  }
  delayMicroseconds(20); //Delay for direction change
  digitalWrite(4, LOW);
  delayMicroseconds(20);
  digitalWrite(4, HIGH);
  //printf(Serial, "Z position: %ld\n", CurrentPosition);
}
