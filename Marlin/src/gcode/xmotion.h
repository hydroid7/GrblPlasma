#pragma once

#include "../inc/MarlinConfig.h"

typedef struct
{
  millis_t jog_accel_timer = 0;
  unsigned long step_timer = 0;
  int step_delay = 0;
  float jog_current_ipm;
  bool jog_cancel = false;
  float jog_begin_position;
  float distance_to_decel;
  float min_pos;
  float max_pos;
  AxisEnum axis_number;
  uint8_t axis_step_len;
  bool axis_invert_dir;
}jog_axis_t;
extern jog_axis_t jog_interface[];
extern float jog_speed;

/*
Global Variables for AVTHC
*/
extern float thc_arc_voltage;
extern float thc_set_voltage;
extern bool thc_enabled;
extern bool print_full_report;

void init_xmotion();
void tick_thc_engine();
void tick_jog_engine();
void tick_xmotion();
void inc_move_z_at_fixed_rate(float, float);
void plan_move_inc(float);
void fire_torch();
void extinguish_torch();
