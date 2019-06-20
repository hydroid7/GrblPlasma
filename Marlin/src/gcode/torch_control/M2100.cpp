#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"
#include "../queue.h"

void probe_z()
{
  while(READ(Z_PROBE_PIN) == HIGH)
  {
    inc_move_at_fixed_rate(-0.010, 10);
  }
}
void GcodeSuite::M2100() {
  if (READ(Z_PROBE_PIN) == LOW)
  {
    SERIAL_ECHOPGM("Abort: Z_PROBE already engaged!");
    SERIAL_EOL();
    return;
  }
  const float pierce_height = parser.floatval('P', 0.150);
  SERIAL_ECHOPAIR_F("Pierce Height: ", pierce_height, 4);
  SERIAL_EOL();
  const float cut_height = parser.floatval('C', 0.175);
  SERIAL_ECHOPAIR_F("Pierce Height: ", cut_height, 4);
  SERIAL_EOL();
  const float pierce_delay = parser.floatval('D', 1.5);
  SERIAL_ECHOPAIR_F("Pierce Delay: ", pierce_delay, 2);
  SERIAL_EOL();
  const int retry = parser.intval('R', 4);
  SERIAL_ECHOPAIR("Retry: ", retry);
  SERIAL_EOL();

  for (int x = 0; x < retry; x++) //Retry 4 times
  {
    probe_z();
    inc_move_at_fixed_rate(0.1 + pierce_height, 10); //Floating head take-up + pirce_height
    fire_torch();
    delay(pierce_delay * 1000); //Delay for pierce
    inc_move_at_fixed_rate(cut_height - pierce_height, 10); //Move to cut height
    if (READ(IN_1_PIN) == LOW)
    {
      break; //We have an ARC-OK signal!
    }
    else
    {
      //We don't have an arc okay signal. Shutoff torch, retract and try!
      SERIAL_ECHOPGM("Arc Transfer failed, retrying!");
      SERIAL_EOL();
      extinguish_torch();
      inc_move_at_fixed_rate(1.00, 25);
    }
  }
 }
