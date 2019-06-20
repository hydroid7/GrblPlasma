#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void GcodeSuite::M3000() {
  SERIAL_ECHOPGM("Jogging X!");
  SERIAL_EOL();
  x_jog_current_ipm = 5.00;
  x_jog_cancel = false;
}
void GcodeSuite::M3001() {
  x_jog_cancel = true;
}
