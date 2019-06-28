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
    if (fabs(jog_interface[0].jog_current_ipm) > 0)
    {
      SERIAL_ECHOPGM("Can't Jog until axis is stopped!");
      SERIAL_EOL();
      return;
    }
    if (direction > 0)
    {
      jog_interface[0].jog_current_ipm = 5.00;
    }
    else
    {
      jog_interface[0].jog_current_ipm = -5.00;
    }
    jog_interface[0].jog_cancel = false;
    jog_interface[0].jog_begin_position = current_position[X_AXIS] / 25.4;
    SERIAL_ECHOPAIR("Jogging Axis: ", axis);
    SERIAL_EOL();
  }
  if (axis == 1)
  {
    if (fabs(jog_interface[1].jog_current_ipm) > 0)
    {
      SERIAL_ECHOPGM("Can't Jog until axis is stopped!");
      SERIAL_EOL();
      return;
    }
    if (direction > 0)
    {
      jog_interface[1].jog_current_ipm = 5.00;
    }
    else
    {
      jog_interface[1].jog_current_ipm = -5.00;
    }
    jog_interface[1].jog_cancel = false;
    jog_interface[1].jog_begin_position = current_position[Y_AXIS] / 25.4;
    SERIAL_ECHOPAIR("Jogging Axis: ", axis);
    SERIAL_EOL();
  }
  if (axis == 2)
  {
    if (fabs(jog_interface[2].jog_current_ipm) > 0)
    {
      SERIAL_ECHOPGM("Can't Jog until axis is stopped!");
      SERIAL_EOL();
      return;
    }
    if (direction > 0)
    {
      jog_interface[2].jog_current_ipm = 5.00;
    }
    else
    {
      jog_interface[2].jog_current_ipm = -5.00;
    }
    jog_interface[2].jog_cancel = false;
    jog_interface[2].jog_begin_position = current_position[Z_AXIS] / 25.4;
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
    jog_interface[0].jog_cancel = true;
  }
  if (axis == 1)
  {
    jog_interface[1].jog_cancel = true;
  }
  if (axis == 2)
  {
    jog_interface[2].jog_cancel = true;
  }
}
