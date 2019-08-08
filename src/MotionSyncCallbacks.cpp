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
  Move torch towards workpiece until the probe input is met and finsish
*/
void probe_torch_and_finish()
{
  torch.move_z_incremental(-10, Z_PROBE_FEEDRATE, stop_on_probe_input, retract_torch_and_finish);
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
  Retract to peirce height (Add the amount of floating head slop) and finish
*/
void retract_torch_and_finish()
{
  torch.move_z_incremental(Z_FLOATING_HEAD_TAKEUP + callback.pierceHeight, Z_RAPID_FEEDRATE, NULL, resume_motion);
}

/*
  Shut off the torch and retract to clearance height
*/
void torch_off_and_retract()
{
  printf(Serial, "(torch_off_and_retract)\n");
  torch.extinguish_torch();
  torch.move_z_incremental(callback.clearanceHeight, Z_RAPID_FEEDRATE, NULL, resume_motion);
}

/*
  Light the torch and delay for pierce
*/
void light_torch_and_pierce_delay()
{
  printf(Serial, "(light_torch_and_pierce_delay)\n");
  torch.fire_torch();
  delay(callback.pierceDelay * 1000);
  if (digitalRead(ARC_OK_PIN) == LOW)
  {
    printf(Serial, "\t-> Has arc okay signal\n");
    torch.move_z_incremental(callback.pierceHeight - callback.pierceHeight, Z_RAPID_FEEDRATE, NULL, resume_motion);
  }
  else
  {
    //This should have a retry routine and notify ncPilot there was a problem
    printf(Serial, "\t-> Does not have arc okay signal\n");
    resume_motion();
    /* Retract to z clearance and try the pierce */
  }
  
}
void resume_motion()
{
  motion.sync_finished();
}