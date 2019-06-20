#pragma once

#include "../inc/MarlinConfig.h"

/*
Global Variables for AVTHC
*/
extern float thc_arc_voltage;
extern float thc_set_voltage;
extern bool thc_enabled;

void tick_xmotion_position_report();
void inc_move_at_fixed_rate(float, float);
void plan_move_inc(float);
void fire_torch();
void extinguish_torch();
