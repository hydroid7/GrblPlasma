

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

/*
Global Variables for AVTHC
*/
float thc_arc_voltage;
float thc_set_voltage;
bool thc_enabled;

/*
Global Variables non-syncronized Jogging
*/
float jog_speed = 300;

millis_t jog_accel_timer = 0;

unsigned long x_step_timer = 0;
int x_step_delay = 0;
float x_jog_current_ipm;
bool x_jog_cancel = false;


/**
 Variables used for automatic position reporting during motion
**/
int32_t last_position[3];
float last_units;
int32_t actual_position[3];
float actual_units;
const float scale[] = DEFAULT_AXIS_STEPS_PER_UNIT;
const float accel[] = DEFAULT_MAX_ACCELERATION;
const float max_feedrate[] = DEFAULT_MAX_FEEDRATE;
unsigned long report_timestamp = 0;
bool has_printed_stop_report;
/**
 Variables used for AVTHC
**/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
const int numReadings = 20; //Number of readings to average from
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
void tick_xmotion()
{
  /* Non-Syncronized Jogging */
  if (millis() > jog_accel_timer + 10)
  {
    jog_accel_timer = millis();
    if (fabs(x_jog_current_ipm) > 0)
    {
      if (x_jog_cancel == false) //Accelerate to target
      {
        if (fabs(x_jog_current_ipm) < jog_speed) //We need to accelerate to jog speed!
        {
          float accel_ips_ps = accel[0] / 25.4;
          float accel_per_ten_ms = accel_ips_ps;
          if (x_jog_current_ipm > 0) //We are a positive jog. Add accel to increase speed
          {
            x_jog_current_ipm += accel_per_ten_ms;
            if (fabs(x_jog_current_ipm) > jog_speed) x_jog_current_ipm = jog_speed;
          }
          else //We are a negative jog. Add negative accel to increase speed
          {
            x_jog_current_ipm += (accel_per_ten_ms * -1);
            if (fabs(x_jog_current_ipm) > jog_speed) x_jog_current_ipm = jog_speed * -1;
          }
        }
      }
      else //Deccelerate to 0
      {
        if (fabs(x_jog_current_ipm) > 0) //We need to deccelerate to a stop!
        {
          float accel_ips_ps = accel[0] / 25.4;
          float accel_per_ten_ms = accel_ips_ps;
          if (x_jog_current_ipm > 0) //We are a positive jog. Subtract accel to decrease speed
          {
            x_jog_current_ipm -= accel_per_ten_ms;
            if (x_jog_current_ipm < 0) x_jog_current_ipm = 0;
          }
          else //We are a negative jog. Subtract negative accel to decrease speed
          {
            x_jog_current_ipm -= (accel_per_ten_ms * -1);
            if (x_jog_current_ipm > 0) x_jog_current_ipm = 0;
          }
        }
      }
    }
  }
  if (x_jog_current_ipm != 0)
  {
    bool x_positive_direction = false;
    if (!INVERT_X_DIR)
    {
      x_positive_direction = true;
    }
    else
    {
      x_positive_direction = false;
    }
    if (x_jog_current_ipm > 0) //Jog Positive
    {
      WRITE(X_DIR_PIN, x_positive_direction);
    }
    else
    {
      WRITE(X_DIR_PIN, !x_positive_direction);
    }
    WRITE(X_ENABLE_PIN, HIGH);
    if (micros() > x_step_timer)
    {
      /*float time_into_jog = (float)(millis() - x_jog_begin) / 1000;
      float accel_ips_ps = accel[0] / 25.4;
      float initial_velocity = 0.05;
      float ips = initial_velocity + (accel_ips_ps * time_into_jog);
      float ipm = ips * 60;
      if (ipm > ((max_feedrate[0] / 25.4) * 60))
      {
        x_jog_finished_accelerating = millis();
        ipm = ((max_feedrate[0] / 25.4) * 60);
      }*/
      float step_delay = ((60 * 1000000) / (scale[0] * 25.4)) / fabs(x_jog_current_ipm);
      x_step_delay = (int)step_delay;
      x_step_timer += x_step_delay;
      WRITE(X_STEP_PIN, HIGH);
      delayMicroseconds(5);
      WRITE(X_STEP_PIN, LOW);
      if (x_jog_current_ipm > 0)
      {
        stepper.set_position(X_AXIS, stepper.position(X_AXIS) + 1);
      }
      else
      {
        stepper.set_position(X_AXIS, stepper.position(X_AXIS) - 1);
      }
      current_position[X_AXIS] = (stepper.position(X_AXIS) / scale[2]);
      planner.set_machine_position_mm(current_position);
    }
  }
 /* Automatic Position Reporting during movement */
 if (millis() > report_timestamp + 100)
 {
   pull_arc_reading();
   actual_position[0] = stepper.position(X_AXIS);
   actual_position[1] = stepper.position(Y_AXIS);
   actual_position[2] = stepper.position(Z_AXIS);
   actual_units = (float)parser.linear_value_to_mm(1);
   if (actual_position[0] != last_position[0] || actual_position[1] != last_position[1] || actual_position[2] != last_position[2] || actual_units != last_units)
   {
     has_printed_stop_report = false;
     float traveled_dist[3];
     traveled_dist[0] = fabs(actual_position[0] - last_position[0]) / scale[0];
     traveled_dist[1] = fabs(actual_position[1] - last_position[1]) / scale[1];
     traveled_dist[2] = fabs(actual_position[2] - last_position[2]) / scale[2];
     float traveled_distance = sqrt((traveled_dist[0] * traveled_dist[0]) + (traveled_dist[1] * traveled_dist[1]) + (traveled_dist[2] * traveled_dist[2]));
     float velocity = traveled_distance * (60000 / (millis() - report_timestamp));
     int precision = 4;
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
     last_units = actual_units;
   }
   else
   {
     if (has_printed_stop_report == false)
     {
       has_printed_stop_report = true;
       SERIAL_ECHOPGM("DRO: STATUS=STOP FEEDRATE=0.000 VELOCITY=0.000");
       SERIAL_EOL();
     }
   }
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
  int32_t stepper_position = stepper.position(Z_AXIS);
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
    stepper_position += move;
    stepper.set_position(Z_AXIS, stepper_position);
    current_position[Z_AXIS] = (stepper_position / scale[2]);
    planner.set_machine_position_mm(current_position);
    tick_xmotion();
  }
}
void plan_move_inc(float dist)
{
  destination[Z_AXIS] = current_position[Z_AXIS] + (dist * 25.4);
  feedrate_mm_s = 254;
  sync_plan_position_e();
  prepare_move_to_destination();
  planner.synchronize();
}
void fire_torch()
{
  extDigitalWrite(SOL1_PIN, HIGH);
}
void extinguish_torch()
{
  extDigitalWrite(SOL1_PIN, LOW);
}
