#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void GcodeSuite::M3000() {
  const float axis = parser.floatval('P', 10);
  const float speed = parser.floatval('S', -10);
  const float direction = parser.floatval('D', 1);
  if (speed != -10)
  {
    SERIAL_ECHOPAIR("Setting Jog Speed: ", speed);
    SERIAL_EOL();
    jog_speed = speed;
  }
  if (axis == 0)
  {
    if (fabs(x_jog_current_ipm) > 0)
    {
      SERIAL_ECHOPGM("Can't Jog until axis is stopped!");
      SERIAL_EOL();
      return;
    }
    if (direction > 0)
    {
      x_jog_current_ipm = 5.00;
    }
    else
    {
      x_jog_current_ipm = -5.00;
    }
    x_jog_cancel = false;
    x_jog_begin_position = current_position[X_AXIS] / 25.4;
    SERIAL_ECHOPAIR("Jogging Axis: ", axis);
    SERIAL_EOL();
  }

}
void GcodeSuite::M3001() {
  const float axis = parser.floatval('P', 10);
  SERIAL_ECHOPAIR("Quit jogging Axis: ", axis);
  SERIAL_EOL();
  if (axis == 0)
  {
    x_jog_cancel = true;
  }
}
