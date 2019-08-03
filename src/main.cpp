#include "Arduino.h"
#include "Machine.h"
#include "MotionPlanner.h"
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
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.4f UNITS=INCH STATUS=RUN\n", pos.x, pos.y, pos.f);
    dro_stop_report = false;
  }
  else if (dro_stop_report == false) //Run this once when motion finishes
  {
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.4f UNITS=INCH STATUS=HALT\n", pos.x, pos.y, pos.f);
    dro_stop_report = true;
  }
  return true; // repeat? true
}
void setup()
{
  //Init everything
  pinMode(LED, OUTPUT);
  gcodes_init();
  motion.init();
  torch.init();

  //Setup timer tasks
  cpu_blink_timer.every(1000, cpu_blink);

  dro_stop_report = false;
  //dro_timer.every(100, dro_output);
}
void loop()
{
  gcodes_tick();
  cpu_blink_timer.tick();
  dro_timer.tick();
}
