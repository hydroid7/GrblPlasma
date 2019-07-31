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

bool cpu_blink(void *)
{
  digitalWrite(LED, !digitalRead(LED)); // toggle the LED
  return true; // repeat? true
}

void setup()
{
  //Init everything
  pinMode(LED, OUTPUT);
  gcodes_init();
  motion.init();

  //Setup timer tasks
  cpu_blink_timer.every(1000, cpu_blink);
}
void loop()
{
  gcodes_tick();
  cpu_blink_timer.tick();
}
