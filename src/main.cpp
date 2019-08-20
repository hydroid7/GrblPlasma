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
}
void loop()
{
  gcodes_tick();
  cpu_blink_timer.tick();
  //dro_timer.tick();
  torch.tick();
  motion.tick();
}
