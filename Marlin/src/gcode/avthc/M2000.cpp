#include "../gcode.h"
#include "../xmotion.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

void GcodeSuite::M2000() {
  const float voltage_setting = parser.floatval('P', 100.00);
  if (voltage_setting < 30)
  {
    SERIAL_ECHOPGM("Turning THC Off!");
    SERIAL_EOL();
    thc_set_voltage = 0;
    thc_enabled = false;
    return;
  }
  else
  {
    SERIAL_ECHOPAIR_F("Setting THC Voltage to ", voltage_setting, 2);
    SERIAL_EOL();
    thc_set_voltage = voltage_setting;
    thc_enabled = true;
  }
}
