#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"
#include "../queue.h"
const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;
bool positive_direction = false;

void fire_torch()
{
  extDigitalWrite(SOL1_PIN, HIGH);
}
void extinguish_torch()
{
  extDigitalWrite(SOL1_PIN, LOW);
}
void move_inc(float dist)
{
  destination[Z_AXIS] = current_position[Z_AXIS] + (dist * 25.4);
  feedrate_mm_s = 254;
  sync_plan_position_e();
  prepare_move_to_destination();
  planner.synchronize();
}
void find_positive_direction()
{
  move_inc(0.001); //Make a small positive move with the planner so we can capture the dir pin for a positive move!
  positive_direction = READ(Z_DIR_PIN);
}
void inc_move_at_fixed_rate(float distance)
{
  WRITE(Z_ENABLE_PIN, HIGH);
  if (distance > 0) //Positive move
  {
    WRITE(Z_DIR_PIN, positive_direction);
  }
  else
  {
    WRITE(Z_DIR_PIN, !positive_direction);
  }
  for (int x = 0; x < (int)scale[3] * 25.4 * fabs(distance); x++)
  {
    WRITE(Z_STEP_PIN, HIGH);
    delayMicroseconds(500);
    WRITE(Z_STEP_PIN, LOW);
    delayMicroseconds(10);
  }
}
void probe_z()
{
  while(READ(Z_PROBE_PIN) == HIGH)
  {
    inc_move_at_fixed_rate(-0.010);
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
  SERIAL_ECHOPAIR_F("Current Z Position: ", current_position[Z_AXIS], 4);
  SERIAL_EOL();

  for (int x = 0; x < 4; x++) //Retry 4 times
  {
    probe_z();
    inc_move_at_fixed_rate(0.1 + pierce_height); //Floating head take-up + pirce_height
    fire_torch();
    delay(pierce_delay * 1000); //Delay for pierce
    inc_move_at_fixed_rate(pierce_height - cut_height); //Move to cut height
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
      inc_move_at_fixed_rate(2.00);
    }
  }
 }
