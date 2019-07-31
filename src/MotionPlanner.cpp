/*
  MotionPlanner
*/
#include "Arduino.h"
#include "MotionPlanner.h"
#include "RingBuf.h"
#include "Gcodes.h"
#include "Machine.h"
#include <IntervalTimer.h>

RingBuf *MoveStack = RingBuf_new(sizeof(struct Move_Data), MOVE_STACK_SIZE);
IntervalTimer motion_timer;

void motion_timer_tick()
{
  motion.motion_tick();
}

MotionPlanner::MotionPlanner()
{
  _Step_Scale.x = 650;
  _Step_Scale.y = 650;

  _Feed_Jerk.x = 0.05;
  _Feed_Jerk.y = 0.05;

  _Feed_Accel.x = 2;
  _Feed_Accel.y = 2;

  _Feed_Sample_Timestamp = 0; //This is in millis()
  _Feedrate_Timestamp = 0; //This is in micros()
  _Feedrate_delay = 500 * 1000;
}
void MotionPlanner::init()
{
  if (motion_timer.begin(motion_timer_tick, 1))
  {
    printf(Serial, "Motion Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "Motion Timer init: FAIL!\n");
  }

}
bool MotionPlanner::push_target(XYZ_Double target)
{
  if (MoveStack->isFull(MoveStack))
  {
    return false;
  }
  else
  {
    struct Move_Data move;
    memset(&move, 0, sizeof(struct Move_Data)); //Zero out the Move_Data, this may be too time consuming? May not be neccisary
    move.target.x = target.x * _Step_Scale.x;
    move.target.y = target.y * _Step_Scale.y;
    move.target.z = 0; //Right now we only need XY for plasma cutting, we'll add in more axis later
    move.target.f = (target.f * 0.0166666) * 1000.0;

    MoveStack->add(MoveStack, &move); //Push it to the stack
    return true;
  }
}
void MotionPlanner::motion_tick()
{
  if (millis() > _Feed_Sample_Timestamp + FEED_RAMP_UPDATE_INTERVAL)
  {

    _Feed_Sample_Timestamp = millis();
  }
  if (micros() > _Feedrate_Timestamp + _Feedrate_delay)
  {

    _Feedrate_Timestamp = micros();
  }
}
