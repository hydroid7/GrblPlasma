

#include "gcode.h"

#include "parser.h"
#include "queue.h"
#include "../module/motion.h"
#include "../module/stepper.h"

#if ENABLED(PRINTCOUNTER)
  #include "../module/printcounter.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../sd/cardreader.h"
  #include "../feature/power_loss_recovery.h"
#endif

#include "../Marlin.h" // for idle() and suspend_auto_report
#include "xmotion.h"

float current_velocity; //Calculated by auto-report, required by AVTHC engine. Always in IPM
float target_velocity; //Calculated by auto-report, required by AVTHC engine. Always in IPM
/*
Global Variables for AVTHC
*/
float thc_arc_voltage;
float thc_set_voltage;
float thc_velocity_tolorance = 5; //Must be within X inches/min to are target velocity before ATHC starts to comp Z height
float thc_voltage_tolorance = 3; //If we are whithin X volts of our target voltage, don't make Z adjustments!
float thc_comp_velocity = 40; //IPM to make adjustments at.
bool thc_enabled;

/*
Global Variables non-syncronized Jogging
*/
float jog_speed = 300;
jog_axis_t jog_interface[3];

/**
 Variables used for automatic position reporting during motion
**/
int32_t last_position[3];
float last_work_offset_position[3];
float last_units;
int32_t actual_position[3];
float actual_units;
const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;
const float accel[] = DEFAULT_MAX_ACCELERATION;
const float max_feedrate[] = DEFAULT_MAX_FEEDRATE;
unsigned long report_timestamp = 0;
unsigned long arc_voltage_timestamp = 0;
int has_printed_stop_report = 0;
bool print_full_report = false;
/**
 Variables used for AVTHC
**/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
const int numReadings = 100; //Number of readings to average from
float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
void pull_arc_reading()
{
 total = total - readings[readIndex];
 readings[readIndex] = analogRead(A21);
 total = total + readings[readIndex];
 readIndex = readIndex + 1;
 if (readIndex >= numReadings) {
   readIndex = 0;
 }
 float actual_voltage = mapfloat((total / numReadings), 0.00, 190.00, 0.00, 9.59); //Calibrated by hooking 9 volt battery to input. This input should be pretty close all the way to 18 volts
 thc_arc_voltage = actual_voltage * 50; //Scaled to 50:1
}
void init_xmotion()
{
  //Setup Jog interface variables
  jog_interface[0].min_pos = X_MIN_POS;
  jog_interface[0].max_pos = X_MAX_POS;
  jog_interface[0].axis_number = X_AXIS;
  jog_interface[0].axis_step_len = 5; //uS
  jog_interface[0].axis_invert_dir = INVERT_X_DIR;

  jog_interface[1].min_pos = Y_MIN_POS;
  jog_interface[1].max_pos = Y_MAX_POS;
  jog_interface[1].axis_number = Y_AXIS;
  jog_interface[1].axis_step_len = 5; //uS
  jog_interface[1].axis_invert_dir = INVERT_Y_DIR;

  jog_interface[2].min_pos = Z_MIN_POS;
  jog_interface[2].max_pos = Z_MAX_POS;
  jog_interface[2].axis_number = Z_AXIS;
  jog_interface[2].axis_step_len = 5; //uS
  jog_interface[2].axis_invert_dir = INVERT_Z_DIR;
}
/*float thc_arc_voltage;
float thc_set_voltage;
float thc_velocity_tolorance = 0.20; //Must be within 20% of target velocity
float thc_comp_velocity = 2; //IPM to make adjustments at
bool thc_enabled;*/

bool IsInTolerance(float a, float b, float t)
{
  float diff;
  if (a > b)
  {
    diff = a - b;
  }
  else
  {
    diff = b - a;
  }
  if (diff <= fabs(t) && diff >= -fabs(t))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void tick_thc_engine()
{
  if (thc_enabled && thc_set_voltage > 30)
  {
    if (IsInTolerance(target_velocity, current_velocity, thc_velocity_tolorance)) //Check to see if we are within our velocity tolorance
    {
      if (READ(IN_1_PIN) == LOW) //Make sure we have our ARC_OK signal, otherwise something is wrong and we should not comp torch!
      {
        if (IsInTolerance(thc_arc_voltage, thc_set_voltage, thc_voltage_tolorance)) //Check to see if we are in tolorance
        {
          //Don't move at all
        }
        else
        {
          if (thc_set_voltage > thc_arc_voltage) //Jog Z positive
          {
            inc_move_z_at_fixed_rate(0.010, thc_comp_velocity);
          }
          else //Jog Z Negative
          {
            inc_move_z_at_fixed_rate(-0.010, thc_comp_velocity);
          }
        }
      }
    }
  }
}
void jog_write_enable_pin(AxisEnum axis, bool val)
{
  if (axis == X_AXIS) WRITE(X_ENABLE_PIN, val);
  if (axis == Y_AXIS)
  {
    WRITE(Y_ENABLE_PIN, val);
    WRITE(Y2_ENABLE_PIN, val);
  }
  if (axis == Z_AXIS) WRITE(Z_ENABLE_PIN, val);
}
void jog_write_step_pin(AxisEnum axis, bool val)
{
  if (axis == X_AXIS) WRITE(X_STEP_PIN, val);
  if (axis == Y_AXIS)
  {
    WRITE(Y_STEP_PIN, val);
    WRITE(Y2_STEP_PIN, val);
  }
  if (axis == Z_AXIS) WRITE(Z_STEP_PIN, val);
}
void jog_write_dir_pin(AxisEnum axis, bool val)
{
  if (axis == X_AXIS) WRITE(X_DIR_PIN, val);
  if (axis == Y_AXIS)
  {
    WRITE(Y_DIR_PIN, val);
    if (INVERT_Y2_VS_Y_DIR == true)
    {
      WRITE(Y2_DIR_PIN, !val);
    }
    else
    {
      WRITE(Y2_DIR_PIN, val);
    }
  }
  if (axis == Z_AXIS) WRITE(Z_DIR_PIN, val);
}
void tick_jog_engine()
{
  /* Non-Syncronized Jogging */
  for (int x = 0; x < 3; x++)
  {
    if (millis() > jog_interface[x].jog_accel_timer + 10)
    {
      jog_interface[x].jog_accel_timer = millis();
      if (fabs(jog_interface[x].jog_current_ipm) > 0)
      {
        float max_jog = jog_speed;
        if (max_jog > max_feedrate[jog_interface[x].axis_number])
        {
          max_jog = (max_feedrate[jog_interface[x].axis_number] / 25.4) * 60;
        }
        if (jog_interface[x].jog_cancel == false) //Accelerate to target
        {
          float accel_ips_ps = accel[jog_interface[x].axis_number] / 25.4;
          float accel_per_ten_ms = accel_ips_ps;
          float hard_limit = jog_interface[x].max_pos;
          if (jog_interface[x].jog_current_ipm < 0) hard_limit = jog_interface[x].min_pos;
          float soft_limit_distance = fabs((fabs(hard_limit) - (stepper.position(jog_interface[x].axis_number) / scale[jog_interface[x].axis_number])) / 25.4);
          if (fabs(jog_interface[x].jog_current_ipm) < max_jog) //We need to accelerate to jog speed!
          {
            if (jog_interface[x].jog_current_ipm > 0) //We are a positive jog. Add accel to increase speed
            {
              //If we continue to accelerate to jog speed, can we deccelerate in time to come to a stop before hitting our positive soft limit?
              jog_interface[x].distance_to_decel = fabs((current_position[jog_interface[x].axis_number] / 25.4) - jog_interface[x].jog_begin_position);
              //SERIAL_ECHOPAIR_F("distance_to_decel: ", distance_to_decel, 4);
              //SERIAL_EOL();
              //SERIAL_ECHOPAIR_F("soft_limit_distance: ", soft_limit_distance, 4);
              //SERIAL_EOL();
              if (jog_interface[x].distance_to_decel > soft_limit_distance)
              {
                //SERIAL_ECHOPGM("Time to stop accelerating and start deccelerating!");
                //SERIAL_EOL();
                jog_interface[x].jog_cancel = true;
              }
              jog_interface[x].jog_current_ipm += accel_per_ten_ms;
              if (fabs(jog_interface[x].jog_current_ipm) > max_jog) jog_interface[x].jog_current_ipm = max_jog;
            }
            else //We are a negative jog. Add negative accel to increase speed
            {
              jog_interface[x].jog_current_ipm += (accel_per_ten_ms * -1);
              if (fabs(jog_interface[x].jog_current_ipm) > max_jog) jog_interface[x].jog_current_ipm = max_jog * -1;
            }
          }
          else //Were at target velocity, just need to check boundries now
          {
            if (jog_interface[x].distance_to_decel > soft_limit_distance)
            {
              jog_interface[x].jog_cancel = true;
            }
          }
        }
        else //Deccelerate to 0
        {
          if (fabs(jog_interface[x].jog_current_ipm) > 0) //We need to deccelerate to a stop!
          {
            float accel_ips_ps = accel[jog_interface[x].axis_number] / 25.4;
            float accel_per_ten_ms = accel_ips_ps;
            if (jog_interface[x].jog_current_ipm > 0) //We are a positive jog. Subtract accel to decrease speed
            {
              jog_interface[x].jog_current_ipm -= accel_per_ten_ms;
              if (jog_interface[x].jog_current_ipm < 0) jog_interface[x].jog_current_ipm = 0;
            }
            else //We are a negative jog. Subtract negative accel to decrease speed
            {
              jog_interface[x].jog_current_ipm -= (accel_per_ten_ms * -1);
              if (jog_interface[x].jog_current_ipm > 0) jog_interface[x].jog_current_ipm = 0;
            }
          }
        }
      }
    }
    if (jog_interface[x].jog_current_ipm != 0)
    {
      bool positive_direction = false;
      if (!jog_interface[x].axis_invert_dir)
      {
        positive_direction = true;
      }
      else
      {
        positive_direction = false;
      }
      if (jog_interface[x].jog_current_ipm > 0) //Jog Positive
      {
        //WRITE(jog_interface[x].axis_dir_pin, positive_direction);
        jog_write_dir_pin(jog_interface[x].axis_number, positive_direction);
      }
      else
      {
        //WRITE(jog_interface[x].axis_dir_pin, !positive_direction);
        jog_write_dir_pin(jog_interface[x].axis_number, !positive_direction);
      }
      jog_write_enable_pin(jog_interface[x].axis_number, HIGH);
      //WRITE(jog_interface[x].axis_enable_pin, HIGH);
      if (micros() > jog_interface[x].step_timer)
      {
          if (fabs(jog_interface[x].jog_current_ipm) < 5)
          {
            if (jog_interface[x].jog_current_ipm > 0)
            {
              jog_interface[x].jog_current_ipm = 5;
            }
            else
            {
              jog_interface[x].jog_current_ipm = -5;
            }
          }
          float step_delay = ((60 * 1000000) / (scale[jog_interface[x].axis_number] * 25.4)) / fabs(jog_interface[x].jog_current_ipm);
          jog_interface[x].step_delay = (int)step_delay;
          jog_interface[x].step_timer = micros() + jog_interface[x].step_delay;
          if (jog_interface[x].jog_current_ipm > 0)
          {
            if (current_position[jog_interface[x].axis_number] + (1 / scale[jog_interface[x].axis_number]) > jog_interface[x].max_pos) //Can't travel past hard stop!
            {
              jog_interface[x].jog_current_ipm = 0;
              return;
            }
            stepper.set_position(jog_interface[x].axis_number, stepper.position(jog_interface[x].axis_number) + 1);
          }
          else
          {
            if (current_position[jog_interface[x].axis_number] - (1 / scale[jog_interface[x].axis_number]) < jog_interface[x].min_pos) //Can't travel past hard stop!
            {
              jog_interface[x].jog_current_ipm = 0;
              return;
            }
            stepper.set_position(jog_interface[x].axis_number, stepper.position(jog_interface[x].axis_number) - 1);
          }
          //WRITE(jog_interface[x].axis_step_pin, HIGH);
          jog_write_step_pin(jog_interface[x].axis_number, HIGH);
          delayMicroseconds(jog_interface[x].axis_step_len);
          //WRITE(jog_interface[x].axis_step_pin, LOW);
          jog_write_step_pin(jog_interface[x].axis_number, LOW);
          current_position[jog_interface[x].axis_number] = (stepper.position(jog_interface[x].axis_number) / scale[jog_interface[x].axis_number]);
          planner.set_machine_position_mm(current_position);
      }
    }
  }
}
void tick_xmotion()
{
  tick_jog_engine();
  if (millis() > arc_voltage_timestamp + 10)
  {
    pull_arc_reading();
    arc_voltage_timestamp = millis();
  }
 /* Automatic Position Reporting during movement */
 if (millis() > report_timestamp + 100)
 {
   actual_position[0] = stepper.position(X_AXIS);
   actual_position[1] = stepper.position(Y_AXIS);
   actual_position[2] = stepper.position(Z_AXIS);
   actual_units = (float)parser.linear_value_to_mm(1);
   int precision = 4;
   if (actual_position[0] != last_position[0] || actual_position[1] != last_position[1] || actual_position[2] != last_position[2] || workspace_offset[0] != last_work_offset_position[0] || workspace_offset[1] != last_work_offset_position[1] || workspace_offset[2] != last_work_offset_position[2] || actual_units != last_units || print_full_report == true)
   {
     print_full_report = false;
     has_printed_stop_report = 0;
     float traveled_dist[3];
     traveled_dist[0] = fabs(actual_position[0] - last_position[0]) / scale[0];
     traveled_dist[1] = fabs(actual_position[1] - last_position[1]) / scale[1];
     traveled_dist[2] = fabs(actual_position[2] - last_position[2]) / scale[2];
     float traveled_distance = sqrt((traveled_dist[0] * traveled_dist[0]) + (traveled_dist[1] * traveled_dist[1]) + (traveled_dist[2] * traveled_dist[2]));
     float velocity = traveled_distance * (60000 / (millis() - report_timestamp));
     current_velocity = velocity / 25.4;
     target_velocity = (feedrate_mm_s) / 0.424;
     if (parser.linear_value_to_mm(1) == 1.0f) precision = 3;
     SERIAL_ECHOPAIR_F("DRO: X_MCS=", (float)(actual_position[0] / scale[0]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" Y_MCS=", (float)(actual_position[1] / scale[1]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" Z_MCS=", (float)(actual_position[2] / scale[2]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" X_WO=", (float)(workspace_offset[0]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" Y_WO=", (float)(workspace_offset[1]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" Z_WO=", (float)(workspace_offset[2]) / actual_units, precision);
     SERIAL_ECHOPAIR_F(" FEEDRATE=", (float)(feedrate_mm_s) / 0.424, 1);
     SERIAL_ECHOPAIR_F(" VELOCITY=", (float)(velocity) / actual_units, 1);
     SERIAL_ECHOPAIR_F(" THC_SET_VOLTAGE=", thc_set_voltage, 2);
     SERIAL_ECHOPAIR_F(" THC_ARC_VOLTAGE=", thc_arc_voltage, 2);
     //SERIAL_ECHOPAIR_F(" X_Cur=", current_position[X_AXIS], 4);
     //
     SERIAL_ECHOPGM(" UNITS=");
     if (parser.linear_value_to_mm(1) == 1.0f)
     {
       SERIAL_ECHO("MM");
     }
     else
     {
       SERIAL_ECHO("INCH");
     }
     SERIAL_ECHOPGM(" STATUS=RUN");
     SERIAL_EOL();
     last_position[0] = actual_position[0];
     last_position[1] = actual_position[1];
     last_position[2] = actual_position[2];
     last_work_offset_position[0] = workspace_offset[0];
     last_work_offset_position[1] = workspace_offset[1];
     last_work_offset_position[2] = workspace_offset[2];
     last_units = actual_units;
   }
   else
   {
     if (has_printed_stop_report < 10)
     {
       has_printed_stop_report++;
       current_velocity = 0;
       SERIAL_ECHOPGM("DRO: STATUS=STOP FEEDRATE=0.000 VELOCITY=0.000");
       SERIAL_ECHOPAIR_F(" X_WO=", (float)(workspace_offset[0]) / actual_units, precision);
       SERIAL_ECHOPAIR_F(" Y_WO=", (float)(workspace_offset[1]) / actual_units, precision);
       SERIAL_ECHOPAIR_F(" Z_WO=", (float)(workspace_offset[2]) / actual_units, precision);
       SERIAL_EOL();
     }
   }
   tick_thc_engine();
   report_timestamp = millis();
 }
 /***********************************************/
}

void inc_move_z_at_fixed_rate(float distance, float feedrate)
{
  bool positive_direction = false;
  if (!INVERT_Z_DIR)
  {
    positive_direction = true;
  }
  else
  {
    positive_direction = false;
  }
  float step_delay = ((60 * 1000000) / (scale[2] * 25.4)) / feedrate;
  //SERIAL_ECHOPAIR_F("Step Delay: ", step_delay, 1);
  //SERIAL_EOL();
  WRITE(Z_ENABLE_PIN, HIGH);
  int move;
  if (distance > 0) //Positive move
  {
    WRITE(Z_DIR_PIN, positive_direction);
    move = 1;
  }
  else
  {
    WRITE(Z_DIR_PIN, !positive_direction);
    move = -1;
  }
  for (int x = 0; x < (int)scale[2] * 25.4 * fabs(distance); x++)
  {
    WRITE(Z_STEP_PIN, HIGH);
    delayMicroseconds((int)step_delay);
    WRITE(Z_STEP_PIN, LOW);
    delayMicroseconds(10);
    //stepper.set_position(Z_AXIS, stepper.position(Z_AXIS) + move);
    //current_position[Z_AXIS] = (stepper.position(Z_AXIS) / scale[2]);
    //planner.set_machine_position_mm(current_position);
    tick_xmotion();
  }
}
/*void plan_move_inc(float dist)
{
  destination[Z_AXIS] = current_position[Z_AXIS] + (dist * 25.4);
  feedrate_mm_s = 254;
  sync_plan_position_e();
  prepare_move_to_destination();
  planner.synchronize();
}*/
void fire_torch()
{
  extDigitalWrite(SOL1_PIN, HIGH);
}
void extinguish_torch()
{
  extDigitalWrite(SOL1_PIN, LOW);
}
