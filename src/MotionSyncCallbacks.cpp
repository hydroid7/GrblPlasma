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
  if (digitalRead(Z_PROBE_PIN) == LOW)
  {
    return true;
  }
  return false;
}

/* condition met callbacks */

/*
  Move torch towards workpiece until the probe input is met
*/
void probe_torch()
{
  printf(Serial, "(probe_torch)\n");
  torch.move_z_incremental(-10, Z_PROBE_FEEDRATE, stop_on_probe_input, retract_torch);
}

/*
  Retract to peirce height (Add the amount of floating head slop)
*/
void retract_torch()
{
  printf(Serial, "(retract_torch)\n");
  torch.move_z_incremental(Z_FLOATING_HEAD_TAKEUP + callback.pierceHeight, Z_RAPID_FEEDRATE, NULL, light_torch_and_pierce_delay);
}

/*
  Light the torch and delay for pierce
*/
void light_torch_and_pierce_delay()
{
  printf(Serial, "(light_torch_and_pierce_delay)\n");
  torch.fire_torch();
  delay(callback.pierceDelay);
  if (digitalRead(ARC_OK_PIN) == LOW) //We have an arc okay signal
  {
    printf(Serial, "\t-> Has arc okay signal\n");
    return; //ncPilot will recieve an okay to send moves
  }
  else
  {
    printf(Serial, "\t-> Does not have arc okay signal\n");
    /* Retract to z clearance and try the pierce */
  }
  
}