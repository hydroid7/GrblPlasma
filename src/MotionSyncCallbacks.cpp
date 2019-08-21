#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "TorchControl.h"
#include "RingBuf.h"
#include "MotionSyncCallbacks.h"
#include "Gcodes.h"

CallbackData callback;
MotionSyncConfig syncConfig;
/* condition check callbacks */
int z_probe_debounce = 0;
bool stop_on_probe_input()
{
  if (digitalRead(Z_PROBE_PIN) == LOW)
  {
    z_probe_debounce++;
  }
  else
  {
    z_probe_debounce = 0;
  }
  if (z_probe_debounce > 10)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/* condition met callbacks */

/*
  Move torch towards workpiece until the probe input is met
*/
void probe_retract_delay()
{
  torch.wait_until(millis() + (1000), probe_torch);
}
void probe_torch()
{
  if (digitalRead(Z_PROBE_PIN) == LOW) //Our probe is closed before probe begins, retract torch!
  {
    torch.move_z_incremental(callback.clearanceHeight, syncConfig.z_rapid_feed, NULL, probe_retract_delay);
  }
  else
  {
    z_probe_debounce = 0;
    torch.move_z_incremental(-10, syncConfig.z_probe_feed, stop_on_probe_input, retract_torch);
  }
}

/*
  Move torch towards workpiece until the probe input is met and finsish
*/
void probe_torch_and_finish()
{
  z_probe_debounce = 0;
  torch.move_z_incremental(-10, syncConfig.z_probe_feed, stop_on_probe_input, retract_torch_and_finish);
}

/*
  Retract to peirce height (Add the amount of floating head slop)
*/
void retract_torch()
{
  torch.move_z_incremental(syncConfig.floating_head_takeup + callback.pierceHeight, syncConfig.z_rapid_feed, NULL, light_torch_and_pierce_delay);
}
/*
  Retract to peirce height (Add the amount of floating head slop) and finish
*/
void retract_torch_and_finish()
{
  torch.move_z_incremental(syncConfig.floating_head_takeup + callback.pierceHeight, syncConfig.z_rapid_feed, NULL, resume_motion);
}

/*
  Shut off the torch and retract to clearance height
*/
void torch_retract()
{
  torch.move_z_incremental(callback.clearanceHeight, syncConfig.z_rapid_feed, NULL, resume_motion);
}
void torch_off_and_retract()
{
  printf(Serial, "(torch_off_and_retract)\n");
  torch.extinguish_torch();
  torch.wait_until(millis() + (1000), torch_retract);
}

void move_to_cut_height()
{
  torch.move_z_incremental(callback.pierceHeight - callback.pierceHeight, syncConfig.z_rapid_feed, NULL, resume_motion);
}
void light_torch_and_pierce_delay()
{
  printf(Serial, "(light_torch_and_pierce_delay)\n");
  torch.fire_torch();
  torch.wait_until(millis() + (callback.pierceDelay * 1000), move_to_cut_height);
}
void resume_motion()
{
  motion.sync_finished();
}
void MotionSync_init()
{
  syncConfig.z_rapid_feed = 2;
  syncConfig.z_probe_feed  = 1.5;
  syncConfig.floating_head_takeup = 0.2;
}