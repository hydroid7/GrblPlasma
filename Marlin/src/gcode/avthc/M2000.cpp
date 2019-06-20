#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void GcodeSuite::M2000() {

  /*destination[Z_AXIS] = 1.00 * 25.4;
  sync_plan_position_e();
  prepare_move_to_destination();
  planner.synchronize();*/

  const float voltage_setting = parser.floatval('P', 100.00);
  if (voltage_setting < 30)
  {
    SERIAL_ECHOPGM("Turning THC Off!");
    SERIAL_EOL();
    thc_set_voltage = 0;
    return;
  }
  else
  {
    SERIAL_ECHOPAIR_F("Setting THC Voltage to ", voltage_setting, 2);
    SERIAL_EOL();
    thc_set_voltage = voltage_setting;
  }
  /*int val = analogRead(A21);
  float actual_voltage = mapfloat((float)val, 0.00, 190.00, 0.00, 9.59); //Calibrated by hooking 9 volt battery to input. This input should be pretty close all the way to 18 volts
  SERIAL_ECHOPAIR_F("Actual Voltage  =", actual_voltage, 2);
  SERIAL_EOL();
  SERIAL_ECHOPAIR_F("Arc Voltage  =", actual_voltage * 50, 2);
  SERIAL_EOL();*/
}
