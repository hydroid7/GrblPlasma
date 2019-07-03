#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void GcodeSuite::M2101() {
  planner.synchronize();
  extDigitalWrite(SOL1_PIN, LOW);
  const float post_delay = parser.floatval('P', 1);
  const float retract_height = parser.floatval('R', 1);
  SERIAL_ECHOPAIR_F("Post Delay: ", post_delay, 2);
  SERIAL_EOL();
  delay(post_delay * 1000);

  SERIAL_ECHOPAIR_F("Retract Height: ", retract_height, 2);
  SERIAL_EOL();
  inc_move_z_at_fixed_rate(retract_height, 30); //Move to retract height!
}
