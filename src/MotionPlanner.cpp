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

  _Feed_Accel.x = 15;
  _Feed_Accel.y = 10;

  _Feed_Sample_Timestamp = 0; //This is in millis()
  _Feedrate_Timestamp = 0; //This is in micros()
  _Feedrate_delay = 500 * 1000;
}
bool MotionPlanner::is_in_motion()
{
  return Motion.run;
}
void MotionPlanner::dump_current_move_to_serial()
{
  printf(Serial, "CurrentPosition.x = %ld, X Scale = %.4f\n", CurrentPosition.x, _Step_Scale.x);
  printf(Serial, "CurrentPosition.y = %ld, Y Scale = %.4f\n", CurrentPosition.y, _Step_Scale.y);
  printf(Serial, "TargetPosition.x = %ld\n", TargetPosition.x);
  printf(Serial, "TargetPosition.y = %ld\n", TargetPosition.y);
  if (Motion.run == true)
  {
    printf(Serial, "Motion.run = true\n");
  }
  else
  {
    printf(Serial, "Motion.run = false\n");
  }
  if (CurrentMove.waiting_for_sync == true)
  {
    printf(Serial, "CurrentMove.waiting_for_sync = true\n");
  }
  else
  {
    printf(Serial, "CurrentMove.waiting_for_sync = false\n");
  }
  
  printf(Serial, "Motion.dx = %ld\n", Motion.dx);
  printf(Serial, "Motion.dy = %ld\n", Motion.dy);
  printf(Serial, "Motion.sx = %ld\n", Motion.sx);
  printf(Serial, "Motion.sy = %ld\n", Motion.sy);
  printf(Serial, "Motion.x_stg = %ld\n", Motion.x_stg);
  printf(Serial, "Motion.y_stg = %ld\n", Motion.y_stg);
  if (Motion.pendingFeedhold == true)
  {
    printf(Serial, "Motion.pendingFeedhold = true\n");
  }
  else
  {
    printf(Serial, "Motion.pendingFeedhold = false\n");
  }
  if (Motion.feedholdActive == true)
  {
    printf(Serial, "Motion.feedholdActive = true\n");
  }
  else
  {
    printf(Serial, "Motion.feedholdActive = false\n");
  }
  printf(Serial, "CurrentMove.target.x = %ld\n", CurrentMove.target.x);
  printf(Serial, "CurrentMove.target.y = %ld\n", CurrentMove.target.y);
  printf(Serial, "CurrentMove.target.z = %ld\n", CurrentMove.target.z);
  printf(Serial, "CurrentMove.target.f = %ld\n", CurrentMove.target.f);
  printf(Serial, "_Feedrate_delay = %ld\n", _Feedrate_delay);
}
void MotionPlanner::init()
{
  _Invert_X_Dir = false;
  _Invert_Y1_Dir = false;
  _Invert_Y2_Dir = false;
  
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
  Motion.pendingFeedhold = false;
  Motion.feedholdActive = false;

  percentage_into_move = 0;

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);

  pinMode(Y1_ENABLE_PIN, OUTPUT);
  pinMode(Y1_DIR_PIN, OUTPUT);
  pinMode(Y1_STEP_PIN, OUTPUT);

  pinMode(Y2_ENABLE_PIN, OUTPUT);
  pinMode(Y2_DIR_PIN, OUTPUT);
  pinMode(Y2_STEP_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y1_ENABLE_PIN, HIGH);
  digitalWrite(Y2_ENABLE_PIN, HIGH);

  if (motion_timer.begin(motion_timer_tick, 1))
  {
    printf(Serial, "Motion Timer init: OK!\n");
  }
  else
  {
    printf(Serial, "Motion Timer init: FAIL!\n");
  }

}
void MotionPlanner::stop()
{
  Motion.run = false;
}
void MotionPlanner::feedhold()
{
  if (Motion.run == true) //Only add a pending feedhold if we are in motion
  {
    Motion.pendingFeedhold = true;
  }
}
void MotionPlanner::run()
{
  CurrentMove.deccel_marker = CurrentMove.accel_marker;
  Motion.run = true;
  Motion.feedholdActive = false;
}
void MotionPlanner::sync_finished()
{
  noInterrupts();
  Motion.dx = 0;
  Motion.dy = 0;
  Motion.sx = 0;
  Motion.sy = 0;
  Motion.err = 0;
  Motion.x_stg = 0;
  Motion.y_stg = 0;
  Motion.run = true;
  CurrentMove.waiting_for_sync = false;
  interrupts();
}
void MotionPlanner::soft_abort()
{
  Motion.pendingSoftAbort = true;
  feedhold(); //This will plan a deceleration then call "abort()" once machine is at MIN_FEED_RATE
}
void MotionPlanner::abort()
{
  //printf(Serial, "(MotionPlanner::abort()) Aborting all moves\n");
  noInterrupts();
  struct Move_Data move;
  while (MoveStack->pull(MoveStack, &move)); //This just clears the stack from the head to the tail
  Motion.dx = 0, Motion.sx = 0;
  Motion.dy = 0, Motion.sy = 0;
  Motion.err = 0;
  Motion.x_stg = 0;
  Motion.y_stg = 0;
  Motion.run = true; //This needs to stay true so the tick can run and do it's job still
  Motion.pendingFeedhold = false;
  Motion.feedholdActive = false;
  CurrentMove.target.x = CurrentPosition.x;
  CurrentMove.target.y = CurrentPosition.y;
  CurrentMove.target.z = CurrentPosition.z;
  CurrentMove.target.f = CurrentPosition.f;
  CurrentMove.waiting_for_sync = false;
  interrupts();
}
XYZ_Long MotionPlanner::get_last_moves_target_steps()
{
  XYZ_Long pos;
  if (MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1) != NULL) //There are moves on the stack
  {
      struct Move_Data *move = (Move_Data*)MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1);
      pos.x = move->target.x;
      pos.y = move->target.y;
  }
  else //There are no moves on the stack
  {
    pos.x = CurrentMove.target.x;
    pos.y = CurrentMove.target.y;
  }
  return pos;
}
XYZ_Double MotionPlanner::get_last_moves_target()
{
  XYZ_Double pos;
  if (MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1) != NULL) //There are moves on the stack
  {
      struct Move_Data *move = (Move_Data*)MoveStack->peek(MoveStack, MoveStack->numElements(MoveStack)-1);
      pos.x = (double)move->target.x / (double)_Step_Scale.x;
      pos.y = (double)move->target.y / (double)_Step_Scale.y;
      pos.f = ((double)move->target.f / FEED_VALUE_SCALE) * 60;
  }
  else //There are no moves on the stack, last moves target is CurrentMove.target
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
XYZ_Double MotionPlanner::get_current_velocity()
{
  XYZ_Double vel;
  vel.x = (double)CurrentVelocity.x;
  vel.y = (double)CurrentVelocity.y;
  vel.f = sqrt(pow((vel.x), 2) + pow(vel.y, 2)) * 60;
  return vel;
}
bool MotionPlanner::push_sync(void (*callback)())
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
    move.move_type = SYNC_MOVE;
    move.sync_callback = callback;
    move.waiting_for_sync = false; //This eeds to be toggled true once the callback get called and toggled false once sync_finish is called
    /* Its important that we push a target the same as the last move or it will break the get_last_target_position chain */
    move.target = get_last_moves_target_steps();

    if (MoveStack->isEmpty(MoveStack) && Motion.run == false) //If the first move is a sync move, don't bother pushing it to the stack, just call it here
    {
      if (move.sync_callback != NULL)
      {
        move.sync_callback();
        CurrentMove.waiting_for_sync = true;
      }
      move.sync_callback = NULL;
    }
    else
    {
      MoveStack->add(MoveStack, &move); //Push the move to the stack!
    }
    return true;
  }
}
bool MotionPlanner::push_target(XYZ_Double target, uint8_t move_type)
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
    move.target.f = (long)(target.f / 60.0) * FEED_VALUE_SCALE;
    move.move_type = move_type;
    XYZ_Double current_position = get_last_moves_target();
    double dominant_accel = _Feed_Accel.x; //Assume X is dominant
    double dominant_dist = abs(target.x - current_position.x);
    if (abs(target.y - current_position.y) > abs(target.x - current_position.x))
    {
      dominant_accel = _Feed_Accel.y; //Y axis is the dominant axis, use it's accel
      dominant_dist = abs(target.y - current_position.y);
    }
    double peak_feedrate = motion_calculate_feed_from_distance(dominant_accel, dominant_dist / 2); //Calculate feed triangle
    if (peak_feedrate > (target.f * 0.0166666)) peak_feedrate = (target.f * 0.0166666);

    move.accel_marker = motion_calculate_accel_marker(dominant_accel, peak_feedrate); //We should not be accelerating after this marker compared to distance in.
    move.deccel_marker = motion_calculate_accel_marker(dominant_accel, peak_feedrate); //We should not be accelerating after this marker compared to distance left.
    //printf(Serial, "Peak Feedrate is (unit/min): %.4f, Accel Marker is: %.4f, Deccel Marker is: %.4f\n", peak_feedrate * 60, move.accel_marker, move.deccel_marker);
    move.entry_velocity = MIN_FEED_RATE;
    move.exit_velocity = MIN_FEED_RATE;
    move.Motion = motion_calculate_target(get_last_moves_target_steps(), move.target);
    MoveStack->add(MoveStack, &move); //Push the move to the stack!
    if (Motion.run == false) //If we are not currently in motion, set our feedrate to min feed and start motion
    {
      motion_set_feedrate(move.entry_velocity);
      Motion.run = true;
    }
    return true;
  }
}
void MotionPlanner::invert_joint_dir(int axis, int value)
{
  bool val = false;
  if (value > 0) val = true;
  switch(axis) {
    case 0:
       _Invert_X_Dir = val;
       break;
    case 1:
       _Invert_Y1_Dir = val;
       break;
    case 2:
       _Invert_Y2_Dir = val;
       break;
  }
}
void MotionPlanner::set_axis_scale(int axis, double value)
{
  switch(axis) {
    case 0:
       _Step_Scale.x = value;
       break;
    case 1:
       _Step_Scale.y = value;
       break;
  }
}
void MotionPlanner::motion_set_feedrate(double feed)
{
  if (Motion.dx == 0 && Motion.dy == 0) return;
  if (feed < 0) return; //Something wen't wrong if this function recieves a negative feedrate!
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
void MotionPlanner::motion_set_target() //Now Obsolete
{
  TargetPosition.x = CurrentMove.target.x;
  TargetPosition.y = CurrentMove.target.y;
  Motion.dx = abs(TargetPosition.x - CurrentPosition.x), Motion.sx = CurrentPosition.x < TargetPosition.x ? 1 : -1;
  Motion.dy = abs(TargetPosition.y - CurrentPosition.y), Motion.sy = CurrentPosition.y < TargetPosition.y ? 1 : -1;
  Motion.err = (Motion.dx>Motion.dy ? Motion.dx : -Motion.dy)/2;
  Motion.x_stg = abs(Motion.dx);
  Motion.y_stg = abs(Motion.dy);
}
Bresenham_Data MotionPlanner::motion_calculate_target(XYZ_Long cur_pos, XYZ_Long tar_pos)
{
  Bresenham_Data ret;
  ret.dx = abs(tar_pos.x - cur_pos.x), ret.sx = cur_pos.x < tar_pos.x ? 1 : -1;
  ret.dy = abs(tar_pos.y - cur_pos.y), ret.sy = cur_pos.y < tar_pos.y ? 1 : -1;
  ret.err = (ret.dx>Motion.dy ? ret.dx : -ret.dy)/2;
  ret.x_stg = abs(ret.dx);
  ret.y_stg = abs(ret.dy);
  return ret;
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
void MotionPlanner::motion_plan_moves_for_continuous_motion_junk()
{
  /*
    Iterate through the moves and comparee this move to last move
    calculate a new exit velocity based on the polar angle of change
  */
  printf(Serial, "Moves on stack: %d\n", MoveStack->numElements(MoveStack));
  int move_index = 0;
  struct Move_Data *last_move = &CurrentMove;
  struct Move_Data *this_move = (Move_Data*)MoveStack->peek(MoveStack, move_index);
  double last_vector_angle = 360; //Zero is 360!
  while(last_move != NULL && this_move != NULL)
  {
    double dominent_axis_jerk = _Feed_Jerk.x;
    double dominent_axis_accel = _Feed_Accel.x;
    double x_dist_inches = abs(last_move->target.x - this_move->target.x) / _Step_Scale.x;
    double y_dist_inches = abs(last_move->target.y - this_move->target.y) / _Step_Scale.y;
    double dominent_axis_dist = x_dist_inches;
    if (y_dist_inches > x_dist_inches)
    {
      dominent_axis_jerk = _Feed_Jerk.y;
      dominent_axis_accel = _Feed_Accel.y;
      dominent_axis_dist = y_dist_inches;
    }
    XYZ_Double last_target;
    last_target.x = last_move->target.x / _Step_Scale.x;
    last_target.y = last_move->target.y / _Step_Scale.y;
    XYZ_Double this_target;
    this_target.x = this_move->target.x / _Step_Scale.x;
    this_target.y = this_move->target.y / _Step_Scale.y;

    printf(Serial, "(continous motion)-> last_target: X%.4f Y%.4f, this_target: X%.4f Y%.4f\n", last_target.x, last_target.y, this_target.x, this_target.y);

    double vector_angle = motion_get_vector_angle(last_target, this_target);
    if (vector_angle == 0) vector_angle += 360;

    double angle_of_change = fabs(last_vector_angle - vector_angle);

    printf(Serial, "Angle of change is: %.4f\n", angle_of_change);
    if (angle_of_change > 180) angle_of_change = 180;
    double exit_velocity = map(angle_of_change, 0, 180, (double)last_move->target.f / FEED_VALUE_SCALE, dominent_axis_jerk);
    if (exit_velocity < dominent_axis_jerk) exit_velocity = dominent_axis_jerk;
    printf(Serial, "New exit/entry velocity is: %.4f\n", exit_velocity);

    double peak_velocity = motion_calculate_feed_from_distance(dominent_axis_accel, dominent_axis_dist / 2.0);
    if (peak_velocity > ((double)last_move->target.f / FEED_VALUE_SCALE)) peak_velocity = ((double)last_move->target.f / FEED_VALUE_SCALE);
    printf(Serial, "Peak Velocity is: %.4f\n", peak_velocity);

    //Figure out how much distance is required to accelerate from peak velocity to exit velocity
    last_move->exit_velocity = exit_velocity;
    last_move->deccel_marker = motion_calculate_accel_marker(dominent_axis_accel, peak_velocity - exit_velocity);
    printf(Serial, "Last Move Decel Marker is: %.4f\n", last_move->deccel_marker);

    this_move->entry_velocity = exit_velocity;
    this_move->accel_marker = motion_calculate_accel_marker(dominent_axis_accel, peak_velocity - exit_velocity);
    printf(Serial, "This Move Accel Marker is: %.4f\n", this_move->accel_marker);

    last_vector_angle = vector_angle;
    //Update move data to next position
    move_index++;
    last_move = (Move_Data*)MoveStack->peek(MoveStack, move_index);
    this_move = (Move_Data*)MoveStack->peek(MoveStack, move_index+1);
  }
}
void MotionPlanner::tick()
{
  if (CurrentMove.waiting_for_sync == true) return; //No need to do anything else here until sync move is finished
  if (CurrentMove.move_type == SYNC_MOVE)
  {
    if (CurrentMove.sync_callback != NULL)
    {
      printf(Serial, "(tick()) Calling sync callback!\n");
      CurrentMove.sync_callback();
      CurrentMove.waiting_for_sync = true;
    }
    CurrentMove.sync_callback = NULL;
    return;
  }
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
        if (CurrentMove.move_type == RAPID_MOVE) //Calculate trapazoidal velocity profile on rapid moves
        {
          if (Motion.pendingFeedhold == true) //Handle our pending feedhold
          {
            Motion.pendingFeedhold = false;
            Motion.feedholdActive = true;
            CurrentMove.feedhold_marker = distance_left; //This is where the feedhold begins
          }
          if (Motion.feedholdActive == false)
          {
            double new_feed_rate;
            bool changed = false;
            if ((distance_in - CurrentMove.feedhold_marker) < CurrentMove.accel_marker) //We should be accelerating
            {
              new_feed_rate = motion_calculate_feed_from_distance(dominent_axis_accel, (distance_in - CurrentMove.feedhold_marker));
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
              if (new_feed_rate > MIN_FEED_RATE && new_feed_rate < CurrentMove.target.f / FEED_VALUE_SCALE) motion_set_feedrate(new_feed_rate);
            }
          }
          else //Feedhold is active, calculate the deccel ramp, stop Motion when feedrate gets under MIN_FEED_RATE
          {
            double new_feed_rate = motion_calculate_feed_from_distance(dominent_axis_accel, CurrentMove.deccel_marker - (CurrentMove.feedhold_marker - distance_left));
            //printf(Serial,"Rapid Feedhold: %.4f\n", new_feed_rate);
            if (new_feed_rate > MIN_FEED_RATE)
            {
              motion_set_feedrate(new_feed_rate);
            }
            else
            {
              Motion.run = false;
              CurrentMove.feedhold_marker = distance_in;
              if (Motion.pendingSoftAbort == true)
              {
                Motion.pendingSoftAbort = false;
                abort();
              }
            }
          }
        }
        else //If we are a line move just do continous motion
        {
          if (Motion.pendingFeedhold == true) //Handle our pending feedhold
          {
            Motion.pendingFeedhold = false;
            Motion.feedholdActive = true;
            Motion.run = false;
            if (Motion.pendingSoftAbort == true)
            {
              Motion.pendingSoftAbort = false;
              printf(Serial, "(MotionPlanner::tick()) Calling abort!\n");
              abort();
            }
          }
          else
          {
            motion_set_feedrate(CurrentMove.target.f / FEED_VALUE_SCALE);
          }
        }
      }
      _Feed_Sample_Timestamp = millis();
    }
  }
}
void MotionPlanner::motion_plan_moves_for_continuous_motion()
{
  /*
    First pass - We need to iterate through all the moves on the stack and save the "current_move" pointer to an array when our vector angle change is more than x degrees.
    Moves that are more than x degrees need to slow down before entering "next_move". Set all the accel and deccel markers to -1 so that there won't be any acceleration or
    decceleration along those moves.

    Second pass -
  */
}
void MotionPlanner::motion_tick()
{
  noInterrupts();
  if (CurrentMove.waiting_for_sync == true) return; //We can't have any syncronized motion while a sync move is happening!
  if (Motion.run == true)
  {
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
        if (MoveStack->pull(MoveStack, &CurrentMove)) //There are pending moves on the stack!
        {
          if (CurrentMove.move_type == SYNC_MOVE)
          {
            /* Since we are a sync move, we need to stop motion and wait for our sync callback to be called. sync_finish() will resume motion */
            printf(Serial, "(motion_tick) - pulled sync move!\n");
            Motion.run = false;
          }
          else
          {
            TargetPosition.x = CurrentMove.target.x;
            TargetPosition.y = CurrentMove.target.y;
            TargetPosition.z = CurrentMove.target.z;
            TargetPosition.f = CurrentMove.target.f;
            Motion.dx = CurrentMove.Motion.dx;
            Motion.dy = CurrentMove.Motion.dy;
            Motion.sx = CurrentMove.Motion.sx;
            Motion.sy = CurrentMove.Motion.sy;
            Motion.err = CurrentMove.Motion.err;
            Motion.x_stg = CurrentMove.Motion.x_stg;
            Motion.y_stg = CurrentMove.Motion.y_stg;
          }
        }
        else
        {
          CurrentVelocity.x = 0;
          CurrentVelocity.y = 0;
          abort();
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
double MotionPlanner::motion_get_relative_angle_between_vectors(XYZ_Double l1p1, XYZ_Double l1p2, XYZ_Double l2p1, XYZ_Double l2p2)
{
  double slope1 = l1p1.y - l1p2.y / l1p1.x - l1p2.x;
  double slope2 = l2p1.y - l2p2.y / l2p1.x - l2p2.x;
  double angle = atan((slope1 - slope2) / (1 - (slope1 * slope2)));
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
    digitalWrite(X_DIR_PIN, _Invert_X_Dir);
  }
  else
  {
    digitalWrite(X_DIR_PIN, !_Invert_X_Dir);
  }
  delayMicroseconds(20); //Delay for direction change
  digitalWrite(X_STEP_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(X_STEP_PIN, HIGH);
}
void MotionPlanner::motion_step_y(int dir)
{
  if (dir > 0)
  {
    digitalWrite(Y1_DIR_PIN, _Invert_Y1_Dir);
    digitalWrite(Y2_DIR_PIN, _Invert_Y2_Dir);
  }
  else
  {
    digitalWrite(Y1_DIR_PIN, !_Invert_Y1_Dir);
    digitalWrite(Y2_DIR_PIN, !_Invert_Y2_Dir);
  }
  delayMicroseconds(20); //Delay for direction change
  digitalWrite(Y1_STEP_PIN, LOW);
  digitalWrite(Y2_STEP_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(Y1_STEP_PIN, HIGH);
  digitalWrite(Y2_STEP_PIN, HIGH);
}
