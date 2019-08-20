#include "Arduino.h"
#include "Machine.h"
#include "MotionPlanner.h"
#include "MotionSyncCallbacks.h"
#include "TorchControl.h"
#include "RingBuf.h"
#include "Gcodes.h"
#include "timer.h"
#include <SD.h>

#include <mk20dx128.h>

const int chipSelect = BUILTIN_SDCARD;

auto cpu_blink_timer = timer_create_default();

bool dro_stop_report;
auto dro_timer = timer_create_default();

auto watchdog_timer = timer_create_default();

bool cpu_blink(void *)
{
  digitalWrite(LED, !digitalRead(LED)); // toggle the LED
  return true; // repeat? true
}
bool dro_output(void *)
{
  XYZ_Double pos = motion.get_current_position();
  if (motion.is_in_motion()) //Are we currently in motion?
  {
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.1f UNITS=INCH THC_ARC_VOLTAGE=%.1f THC_SET_VOLTAGE=%.1f STATUS=RUN\n", pos.x, pos.y, pos.f, torch.get_arc_voltage(), torch.get_set_voltage());
    dro_stop_report = false;
  }
  else
  {
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.1f UNITS=INCH THC_ARC_VOLTAGE=%.4f THC_SET_VOLTAGE=%.4f STATUS=HALT\n", pos.x, pos.y, pos.f, torch.get_arc_voltage(), torch.get_set_voltage());
  }
  
  /*else if (dro_stop_report == false) //Run this once when motion finishes
  {
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.1f UNITS=INCH THC_ARC_VOLTAGE=%.4f THC_SET_VOLTAGE=%.4f STATUS=HALT\n", pos.x, pos.y, pos.f, torch.get_arc_voltage(), torch.get_set_voltage());
    dro_stop_report = true;
  }*/
  return true; // repeat? true
}

/*
  This timers function is to turn the torch off in the event that something happens
  betwen ncPilot and the controller causing a condition where the torch is on but
  move stack runs dry and motion stops... If this happens, we need to turn the torch off!
*/
XYZ_Double last_pos;
bool watchdog_check(void *)
{
  XYZ_Double current_pos = motion.get_current_position();
  if (current_pos.x == last_pos.x && current_pos.y == last_pos.y && torch.get_torch_state() == true)
  {
    //There has been no movement in the last three seconds and the torch is on, this is error!
    torch.extinguish_torch();
  }
  last_pos = current_pos;
}
void setup()
{
  //Init everything
  pinMode(LED, OUTPUT);
  gcodes_init();
  motion.init();
  torch.init();
  MotionSync_init();

  //Setup timer tasks
  cpu_blink_timer.every(1000, cpu_blink);

  dro_stop_report = false;
  dro_timer.every(100, dro_output);

  last_pos = motion.get_current_position();
  watchdog_timer.every(6000, watchdog_check);
}
void loop()
{
  gcodes_tick();
  cpu_blink_timer.tick();
  dro_timer.tick();
  torch.tick();
  motion.tick();
}
