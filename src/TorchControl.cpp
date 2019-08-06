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
THC_Data thc;
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
  //Test

  thc.arc_voltage = 0;
  thc.set_voltage = 0;
  //thc.velocity_tolorance = 0.083; //Must be within X inches/min to are target velocity before ATHC starts to comp Z height
  thc.voltage_tolorance = 2; //If we are whithin X volts of our target voltage, don't make Z adjustments!
  thc.comp_velocity = 0.166666; //IPS to make adjustments at.
  thc.enabled = true;
}
void TorchControl::init()
{
  pinMode(Z_STEP_PIN, OUTPUT); //Z Step
  pinMode(Z_DIR_PIN, OUTPUT); //Z Dir
  pinMode(ARC_START_PIN, OUTPUT); //Arc Start
  pinMode(Z_PROBE_PIN, INPUT); //Z Probe
  pinMode(ARC_VOLTAGE_PIN, INPUT);

  if (move_timer.begin(move_timer_tick, 10))
  {
    printf(Serial, "move_timer Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "move_timer Timer init: FAIL!\n");
  }
  _Feedrate_delay = cycle_frequency_from_feedrate(MIN_FEED_RATE);

  thc.numReadings = NUMBER_OF_READINGS;
  for (int x = 0; x < NUMBER_OF_READINGS; x++) thc.readings[x] = 0;
  thc.readIndex = 0;
  thc.total = 0;
  thc.average = 0;
  thc.torch_on = false;
}
void TorchControl::cancel()
{
  run = false;
}
double TorchControl::get_arc_voltage()
{
  return thc.arc_voltage;
}
double TorchControl::get_set_voltage()
{
  return thc.set_voltage;
}
void TorchControl::set_arc_voltage(double volts)
{
  thc.set_voltage = volts;
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
double TorchControl::mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
bool TorchControl::IsInTolerance(double a, double b, double t)
{
  double diff;
  if (a > b)
  {
    diff = a - b;
  }
  else
  {
    diff = b - a;
  }
  if (diff <= fabs(t) && diff >= -fabs(t))
  {
    return true;
  }
  else
  {
    return false;
  }
}
void TorchControl::sample_voltage()
{
  thc.total = thc.total - thc.readings[thc.readIndex];
  thc.readings[thc.readIndex] = analogRead(ARC_VOLTAGE_PIN);
  thc.total = thc.total + thc.readings[thc.readIndex];
  thc.readIndex++;
  if (thc.readIndex >= thc.numReadings) {
    thc.readIndex = 0;
  }
  double adc_reading = (double)(thc.total / thc.numReadings);
  /*
    603 adc = 0V
    330 adc = 3V
  */
  double actual_voltage = mapdouble(adc_reading, 603, 0, 0.00, 7);
  thc.arc_voltage = actual_voltage * 50.00; //Scaled to 50:1
}
void TorchControl::tick()
{
  sample_voltage();
  if (thc.torch_on == true && thc.enabled == true && thc.set_voltage > 10 && millis() > (torch_fired_timestamp + (3 * 1000)))
  {
    if (digitalRead(ARC_OK_PIN) == LOW) //Make sure we have our ARC_OK signal, otherwise something is wrong and we should not comp torch!
    {
      if (IsInTolerance(thc.arc_voltage, thc.set_voltage, thc.voltage_tolorance)) //Check to see if we are in tolorance
      {
        torch.cancel(); //Cancel our torch movement
      }
      else
      {
        if (thc.set_voltage > thc.arc_voltage) //Jog Z positive
        {
          torch.move_z_incremental(1, thc.comp_velocity, NULL, NULL);
        }
        else //Jog Z Negative
        {
          torch.move_z_incremental(-1, thc.comp_velocity, NULL, NULL);
        }
      }
    }
  }
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
        StepsToGo = 0;
        ConditionCallback = NULL; //make sure the callback is cleared so it only runs once
        //The callback is responsable for stopping motion
        if (ConditionMetCallback != NULL)
        {
          ConditionMetCallback();
        }
      }
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
  thc.torch_on = true;
  torch_fired_timestamp = millis();
  digitalWrite(28, HIGH);
}
void TorchControl::extinguish_torch()
{
  thc.torch_on = false;
  digitalWrite(28, LOW);
}
