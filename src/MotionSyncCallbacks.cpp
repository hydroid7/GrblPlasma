#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "TorchControl.h"
#include "RingBuf.h"
#include "MotionSyncCallbacks.h"
#include "Gcodes.h"

CallbackData callback;

/* condition check callbacks */
bool stop_on_probe_input()
{
  if (Z_PROBE_PIN == LOW)
  {
    return true;
  }
  return false;
}

/* condition met callbacks */
void probe_torch()
{
  torch.move_z_incremental(-10, Z_PROBE_FEEDRATE, stop_on_probe_input, retract_torch);
}
void retract_torch()
{
  torch.move_z_incremental(Z_FLOATING_HEAD_TAKEUP + 0.0625, Z_RAPID_FEEDRATE, NULL, light_torch_and_pierce_delay);
}
void light_torch_and_pierce_delay()
{
  
}