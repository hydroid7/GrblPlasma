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
  _Step_Scale.x = 518;
  _Step_Scale.y = 518;

  _Feed_Jerk.x = 0.05;
  _Feed_Jerk.y = 0.05;

  _Feed_Accel.x = 7;
  _Feed_Accel.y = 6;

  _Feed_Sample_Timestamp = 0; //This is in millis()
  _Feedrate_Timestamp = 0; //This is in micros()
  _Feedrate_delay = 500 * 1000;
}
bool MotionPlanner::is_in_motion()
{
  return Motion.run;
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

  percentage_into_move = 0;

  if (motion_timer.begin(motion_timer_tick, 1))
  {
    printf(Serial, "Motion Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "Motion Timer init: FAIL!\n");
  }

}
XYZ_Double MotionPlanner::get_last_moves_target()
{
  XYZ_Double pos;
  if (MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1) != NULL) //There moves on the stack
  {
      struct Move_Data *move = (Move_Data*)MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1);
      pos.x = (double)move->target.x / (double)_Step_Scale.x;
      pos.y = (double)move->target.y / (double)_Step_Scale.y;
      pos.f = ((double)move->target.f / FEED_VALUE_SCALE) * 60;
  }
  else //There are no moves on the stack
  {
    pos.x = (double)CurrentMove.target.x / (double)_Step_Scale.x;
    pos.y = (double)CurrentMove.target.y / (double)_Step_Scale.y;
    pos.f = ((double)CurrentMove.target.f / FEED_VALUE_SCALE) * 60;
  }
  return pos;
}
XYZ_Double MotionPlanner::get_current_position()
{
  XYZ_Double pos;
  pos.x = (double)CurrentPosition.x / (double)_Step_Scale.x;
  pos.y = (double)CurrentPosition.y / (double)_Step_Scale.y;
  pos.f = sqrt(pow((CurrentVelocity.x), 2) + pow(CurrentVelocity.y, 2)) * 60;
  return pos;
}
bool MotionPlanner::push_target(XYZ_Double target)
{
  if (MoveStack->isFull(MoveStack))
  {
    //We need to setup a system that will wait for the stack to have space available agian and send an OK
    return false;
  }
  else
  {
    struct Move_Data move;
    memset(&move, 0, sizeof(struct Move_Data)); //Zero out the Move_Data, this may be too time consuming? May not be neccisary
    move.target.x = target.x * _Step_Scale.x;
    move.target.y = target.y * _Step_Scale.y;
    move.target.z = 0; //Right now we only need XY for plasma cutting, we'll add in more axis later
    move.target.f = (target.f * 0.0166666) * FEED_VALUE_SCALE;

    //If we are the first move on the stack, make sure we calculate the ramp_map before we start motion otherwise motion could outrun the calculated values on small moves
    XYZ_Double current_position = get_last_moves_target();
    double dominant_accel = _Feed_Accel.x; //Assume X is dominant
    if (abs(target.y - current_position.y) > abs(target.x - current_position.x)) dominant_accel = _Feed_Accel.y; //Y axis is the dominant axis, use it's accel
    move.accel_marker = motion_calculate_accel_marker(dominant_accel, (target.f * 0.0166666)); //We should not be accelerating after this marker compared to distance in.
    move.deccel_marker = motion_calculate_accel_marker(dominant_accel, (target.f * 0.0166666)); //We should not be accelerating after this marker compared to distance left.
    move.entry_velocity = MIN_FEED_RATE;
    move.exit_velocity = MIN_FEED_RATE;
    int move_index = MoveStack->add(MoveStack, &move); //Push the move to the stack!
    if (Motion.run == false) //If we are not currently in motion, set our feedrate to min feed
    {
      motion_set_feedrate(move.entry_velocity); //Minimum feed of 1 inch/min
    }
    Motion.run = true; //Start motion after the ramp map has been calculated
    return true;
  }
}
void MotionPlanner::motion_set_feedrate(double feed)
{
  if (Motion.dx == 0 && Motion.dy == 0) return;
  //if (feed == 0) return; //Can't caclulate a feedrate of zero!
  double x_dist_inches = (double)Motion.dx / _Step_Scale.x;
  double y_dist_inches = (double)Motion.dy / _Step_Scale.y;
  if (feed == 0)
  {
    if (x_dist_inches > y_dist_inches)
    {
      feed = _Feed_Jerk.x;
    }
    else
    {
      feed = _Feed_Jerk.y;
    }
  }
  double cartesion_distance = sqrt(pow((x_dist_inches), 2) + pow(y_dist_inches, 2));
  double move_duration_sec = cartesion_distance / feed;
  long number_of_cycles = Motion.dx;
  if (Motion.dy > number_of_cycles) number_of_cycles = Motion.dy;
  _Feedrate_delay = (move_duration_sec * 1000.0 * 1000.0) / (double)number_of_cycles;
  CurrentVelocity.x = x_dist_inches / move_duration_sec;
  CurrentVelocity.y = y_dist_inches / move_duration_sec;
  //printf(Serial, "Feed Delay is: %ld\n", _Feedrate_delay);
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
double MotionPlanner::motion_calculate_accel_marker(double accel_rate, double target_velocity)
{
  double seconds_to_target = (target_velocity - MIN_FEED_RATE) / accel_rate;
  return (MIN_FEED_RATE * seconds_to_target) + (0.5 * accel_rate * pow(seconds_to_target, 2));
}
double MotionPlanner::motion_calculate_feed_from_distance(double accel_rate, double distance_into_move)
{
  double accel_time = sqrt((0.5 * accel_rate) * distance_into_move) * (1.0/(0.5 * accel_rate));
  return (accel_rate * accel_time);
}
void MotionPlanner::motion_plan_moves_for_continuous_motion()
{
  /*
    Iterate through the moves and comparee this move to last move
    calculate a new exit velocity based on the polar angle of change
  */
  double last_vector_polar_angle = 0;
  for (int x = 0; x < MoveStack->numElements(MoveStack); x++)
  {
    if (MoveStack->peek(MoveStack, x) != NULL) //This element exists
    {
      struct Move_Data *last_move;
      struct Move_Data *this_move;
      if (x == 0) //Last move is the current move that's executing
      {
        last_move = &CurrentMove;
        this_move = (Move_Data*)MoveStack->peek(MoveStack, x);
      }
      else
      {
        last_move = (Move_Data*)MoveStack->peek(MoveStack, x-1);
        this_move = (Move_Data*)MoveStack->peek(MoveStack, x);
      }
      double dominent_axis_jerk = _Feed_Jerk.x; //Assume X then update to Y if its the dominent axis
      double dominent_axis_accel = _Feed_Accel.x;
      double x_dist_inches = abs(last_move->target.x - this_move->target.x) / _Step_Scale.x;
      double y_dist_inches = abs(last_move->target.y - this_move->target.y) / _Step_Scale.y;
      if (y_dist_inches > x_dist_inches)
      {
        dominent_axis_jerk = _Feed_Jerk.y;
        dominent_axis_accel = _Feed_Accel.y;
      }
      XYZ_Double last_target;
      last_target.x = last_move->target.x / _Step_Scale.x;
      last_target.y = last_move->target.y / _Step_Scale.y;
      XYZ_Double this_target;
      this_target.x = this_move->target.x / _Step_Scale.x;
      this_target.y = this_move->target.y / _Step_Scale.y;

      double vector_polar_angle = motion_get_vector_angle(last_target, this_target);
      printf(Serial, "(continous motion)-> last_target: X%.4f Y%.4f, this_target: X%.4f Y%.4f, vector_polar_angle: %.4f\n", last_target.x, last_target.y, this_target.x, this_target.y, vector_polar_angle);
      double angle_of_change = abs(vector_polar_angle - last_vector_polar_angle);
      printf(Serial, "Angle of change is: %.4f\n", angle_of_change);
      if (angle_of_change > 180) angle_of_change = 180;
      double exit_velocity = map(angle_of_change, 0, 180, (double)last_move->target.f / FEED_VALUE_SCALE, dominent_axis_jerk);
      if (exit_velocity < dominent_axis_jerk) exit_velocity = dominent_axis_jerk;
      printf(Serial, "New exit/entry velocity is: %.4f\n", exit_velocity);

      last_move->exit_velocity = exit_velocity;
      last_move->deccel_marker = motion_calculate_accel_marker(dominent_axis_accel, exit_velocity);
      printf(Serial, "Last Move Decel Marker is: %.4f\n", last_move->deccel_marker);

      this_move->entry_velocity = exit_velocity;
      this_move->accel_marker = motion_calculate_accel_marker(dominent_axis_accel, exit_velocity);
      printf(Serial, "This Move Accel Marker is: %.4f\n", this_move->accel_marker);

      last_vector_polar_angle = vector_polar_angle;
    }
  }
}
void MotionPlanner::motion_tick()
{
  noInterrupts();
  if (Motion.run == true)
  {
    if (millis() > _Feed_Sample_Timestamp + FEED_RAMP_UPDATE_INTERVAL)
    {
      double dominent_axis_distance = Motion.dx;
      double dominent_axis_stg = Motion.x_stg;
      double dominent_axis_scale = _Step_Scale.x;
      double dominent_axis_accel = _Feed_Accel.x;
      if (Motion.dy > Motion.dx)
      {
        dominent_axis_distance = Motion.dy;
        dominent_axis_stg = Motion.y_stg;
        dominent_axis_scale = _Step_Scale.y;
        dominent_axis_accel = _Feed_Accel.y;
      }
      if (dominent_axis_stg > 0)
      {
        double distance_left = dominent_axis_stg / dominent_axis_scale;
        double distance_in = (dominent_axis_distance / dominent_axis_scale) - distance_left;
        //printf(Serial, "Distance in: %.4f, Distance Left: %.4f\n", distance_in, distance_left);
        /*
          Use our distance in to deterine which marker we should be using. If accelerating or deccelerating, calculate that value and call motion_set_feedrate
        */
        double new_feed_rate;
        bool changed = false;
        if (distance_in < CurrentMove.accel_marker) //We should be accelerating
        {
          new_feed_rate = motion_calculate_feed_from_distance(dominent_axis_accel, distance_in);
          changed = true;
        }
        if (distance_left < CurrentMove.deccel_marker) //We should be deccelerating
        {
          new_feed_rate = motion_calculate_feed_from_distance(dominent_axis_accel, distance_left);
          changed = true;
        }
        if (changed)
        {
          //printf(Serial, "New feedrate: %.4f\n", new_feed_rate);
          motion_set_feedrate(new_feed_rate);
        }
      }
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
          motion_plan_moves_for_continuous_motion();
          motion_set_target();
        }
        else
        {
          CurrentVelocity.x = 0;
          CurrentVelocity.y = 0;
          Motion.run = false;
        }
      }
      _Feedrate_Timestamp = micros();
    }
  }
  interrupts();
}
double MotionPlanner::motion_get_vector_angle(XYZ_Double p1, XYZ_Double p2)
{
  double angle = to_degrees(atan2(p1.y - p2.y, p1.x - p2.x));
  angle += 180;
  if (angle >= 360) angle -= 360;
  return angle;
}
double MotionPlanner::to_degrees(double radians)
{
  return radians * 180 / 3.14159;
}
double MotionPlanner::to_radians(double degrees)
{
  return degrees * 3.14159 / 180;
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
    digitalWrite(3, LOW);
    digitalWrite(10, LOW);
  }
  else
  {
    digitalWrite(3, HIGH);
    digitalWrite(10, HIGH);
  }
  digitalWrite(2, HIGH);
  digitalWrite(9, HIGH);
  delayMicroseconds(5);
  digitalWrite(2, LOW);
  digitalWrite(9, LOW);
}
