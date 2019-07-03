#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

void GcodeSuite::M2001() {
  planner.synchronize();
  SERIAL_ECHOPGM("THC Enabled!");
  SERIAL_EOL();
  thc_enabled = true;
}
void GcodeSuite::M2002() {
  planner.synchronize();
  SERIAL_ECHOPGM("THC Disabled!");
  SERIAL_EOL();
  thc_enabled = false;
}
void GcodeSuite::M2003() {
  if (thc_enabled)
  {
    SERIAL_ECHOPGM("THC is currently enabled!");
    SERIAL_EOL();
  }
  else
  {
    SERIAL_ECHOPGM("THC is currently disabled!");
    SERIAL_EOL();
  }
  SERIAL_ECHOPAIR_F("Voltage is set to ", thc_set_voltage, 2);
  SERIAL_EOL();
  SERIAL_ECHOPAIR_F("Current Measured Arc Voltage is ", thc_arc_voltage, 2);
  SERIAL_EOL();
}
