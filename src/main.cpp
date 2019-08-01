#include "Arduino.h"
#include "Machine.h"
#include "MotionPlanner.h"
#include "RingBuf.h"
#include "Gcodes.h"
#include "timer.h"
#include <SD.h>

#include <mk20dx128.h>

const int chipSelect = BUILTIN_SDCARD;

auto cpu_blink_timer = timer_create_default();
auto dro_timer = timer_create_default();

bool cpu_blink(void *)
{
  digitalWrite(LED, !digitalRead(LED)); // toggle the LED
  return true; // repeat? true
}
bool dro_output(void *)
{
  if (motion.is_in_motion()) //Are we currently in motion?
  {
    XYZ_Double pos = motion.get_current_position();
    printf(Serial, "DRO: X_MCS=%.4f Y_MCS=%.4f VELOCITY=%.4f\n", pos.x, pos.y, pos.f);
  }
  return true; // repeat? true
}
void setup()
{
  //Init everything
  pinMode(LED, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(9, OUTPUT);

  gcodes_init();
  motion.init();

  //Setup timer tasks
  cpu_blink_timer.every(1000, cpu_blink);
  dro_timer.every(100, dro_output);
}
void loop()
{
  gcodes_tick();
  cpu_blink_timer.tick();
  dro_timer.tick();
}
