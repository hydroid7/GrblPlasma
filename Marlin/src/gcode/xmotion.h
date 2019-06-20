#pragma once

#include "../inc/MarlinConfig.h"

/*
Global Variables for AVTHC
*/
extern float thc_arc_voltage;
extern float thc_set_voltage;
extern bool thc_enabled;

/*
Global Variables non-syncronized Jogging
*/

extern float x_jog_current_ipm;
extern bool x_jog_cancel;

void tick_xmotion();
void inc_move_z_at_fixed_rate(float, float);
void plan_move_inc(float);
void fire_torch();
void extinguish_torch();
