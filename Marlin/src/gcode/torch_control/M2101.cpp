#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/stepper.h"

const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void GcodeSuite::M2101() {
  extDigitalWrite(SOL1_PIN, LOW);
}
