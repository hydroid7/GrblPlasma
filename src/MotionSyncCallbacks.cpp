#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "TorchControl.h"
#include "RingBuf.h"
#include "MotionSyncCallbacks.h"
#include "Gcodes.h"

void probe_and_fire_torch()
{
  printf(Serial, "Motion has halted, now touching off torch!\n");
  delay(500);
  printf(Serial, "Finished!\n");
}