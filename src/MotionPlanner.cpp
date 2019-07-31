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
  CurrentMove.target.x = 0;
  CurrentMove.target.y = 0;
  CurrentMove.target.z = 0;
  CurrentMove.target.f = 0;

  CurrentPosition.x = 0;
  CurrentPosition.y = 0;
  CurrentPosition.z = 0;
  CurrentPosition.f = 0;

  TargetPosition.x = 0;
  TargetPosition.y = 0;
  TargetPosition.z = 0;
  TargetPosition.f = 0;

  Motion.dx = 0;
  Motion.dy = 0;
  Motion.sx = 0;
  Motion.sy = 0;
  Motion.err = 0;
  Motion.x_stg = 0;
  Motion.y_stg = 0;
  Motion.run = true;

  if (motion_timer.begin(motion_timer_tick, 1))
  {
    printf(Serial, "Motion Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "Motion Timer init: FAIL!\n");
  }

}
XYZ_Double MotionPlanner::get_current_position()
{
  XYZ_Double pos;
  pos.x = (double)CurrentPosition.x / (double)_Step_Scale.x;
  pos.y = (double)CurrentPosition.y / (double)_Step_Scale.y;
  return pos;
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
void MotionPlanner::motion_set_feedrate(double feed)
{
  if (Motion.dx == 0 && Motion.dy == 0) return;
  double x_dist_inches = (double)Motion.dx / _Step_Scale.x;
  double y_dist_inches = (double)Motion.dy / _Step_Scale.y;
  double cartesion_distance = sqrt(pow((x_dist_inches), 2) + pow(y_dist_inches, 2));
  double move_duration_sec = cartesion_distance / feed;
  long number_of_cycles = Motion.dx;
  if (Motion.dy > number_of_cycles) number_of_cycles = Motion.dy;
  _Feedrate_delay = (move_duration_sec * 1000.0 * 1000.0) / (double)number_of_cycles;
  CurrentVelocity.x = x_dist_inches / move_duration_sec;
  CurrentVelocity.y = y_dist_inches / move_duration_sec;
  printf(Serial, "Feed Delay is: %ld\n", _Feedrate_delay);
}
void MotionPlanner::motion_set_target()
{
  TargetPosition.x = CurrentMove.target.x;
  TargetPosition.y = CurrentMove.target.y;
  Motion.dx = abs(TargetPosition.x - CurrentPosition.x), Motion.sx = CurrentPosition.x < TargetPosition.x ? 1 : -1;
  Motion.dy = abs(TargetPosition.y - CurrentPosition.y), Motion.sy = CurrentPosition.y < TargetPosition.y ? 1 : -1;
  Motion.err = (Motion.dx>Motion.dy ? Motion.dx : -Motion.dy)/2;
  Motion.x_stg = abs(Motion.dx);
  Motion.y_stg = abs(Motion.dy);
}
void MotionPlanner::motion_tick()
{
  noInterrupts();
  if (Motion.run == true)
  {
    if (millis() > _Feed_Sample_Timestamp + FEED_RAMP_UPDATE_INTERVAL)
    {

      _Feed_Sample_Timestamp = millis();
    }
    if (micros() > _Feedrate_Timestamp + _Feedrate_delay)
    {
      int dominent_axis_stg = Motion.x_stg;
      if (Motion.dy > Motion.dx)
      {
        dominent_axis_stg = Motion.y_stg;
      }
      if (dominent_axis_stg > 0)
      {
        //Step our axis one tick at a time!
        if (Motion.err > -Motion.dx) { Motion.err -= Motion.dy; CurrentPosition.x += Motion.sx; Motion.x_stg--; motion_step_x(Motion.sx); }
        if (Motion.err < Motion.dy) { Motion.err += Motion.dx; CurrentPosition.y += Motion.sy; Motion.y_stg--; motion_step_y(Motion.sy); }
      }
      else
      {
        if (MoveStack->numElements(MoveStack) > 0) //There are pending moves on the stack!
        {
          MoveStack->pull(MoveStack, &CurrentMove);
          motion_set_target();
          motion_set_feedrate((double)CurrentMove.target.f / 1000); //This should actually be the jerk value for the dominent axis
        }
        else
        {
          CurrentVelocity.x = 0;
          CurrentVelocity.y = 0;
          //Motion.run = false;
        }
      }
      _Feedrate_Timestamp = micros();
    }
  }
  interrupts();
}
void MotionPlanner::motion_step_x(int dir)
{
  if (dir > 0)
  {
    digitalWrite(1, HIGH);
  }
  else
  {
    digitalWrite(1, LOW);
  }
  digitalWrite(0, HIGH);
  delayMicroseconds(5);
  digitalWrite(0, LOW);
}
void MotionPlanner::motion_step_y(int dir)
{
  if (dir > 0)
  {
    digitalWrite(3, HIGH);
    digitalWrite(10, HIGH);
  }
  else
  {
    digitalWrite(3, LOW);
    digitalWrite(10, HIGH);
  }
  digitalWrite(2, HIGH);
  digitalWrite(9, HIGH);
  delayMicroseconds(5);
  digitalWrite(2, LOW);
  digitalWrite(9, LOW);
}
