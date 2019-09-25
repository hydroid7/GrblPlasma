#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "TorchControl.h"
#include "MotionSyncCallbacks.h"
#include "RingBuf.h"
#include "Gcodes.h"

bool PendingOkay;

SerialCommand sCmd;
MotionPlanner motion;
TorchControl torch;

void unrecognized(const char *command)
{
  printf(Serial, "\"%s\" is not a supported command!\n", command);
  OkayToSend();
}
void fire_torch()
{
  //Load some default values that will work most of the time
  callback.pierceHeight = 0.0625;
  callback.pierceDelay = 1.5;
  callback.cutHeight = 0.085;

  char *pierceHeight = sCmd.next();
  char *pierceDelay = sCmd.next();
  char *cutHeight = sCmd.next();

  if (pierceHeight != NULL) callback.pierceHeight = atof(pierceHeight);
  if (pierceDelay != NULL) callback.pierceDelay = atof(pierceDelay);
  if (cutHeight != NULL) callback.cutHeight = atof(cutHeight);

  motion.push_sync(&probe_torch);
  OkayToSend();
}
void torch_off()
{
  callback.clearanceHeight = 2;
  motion.push_sync(&torch_off_and_retract);
  OkayToSend();
}
void set_voltage()
{
  char *voltage = sCmd.next();

  if (voltage != NULL)
  {
    double volts = atof(voltage);
    printf(Serial, "Setting ATHC target voltage to %.4f\n", volts);
    torch.set_arc_voltage(volts);
  }
  else
  {
    printf(Serial, "Command usage: set_voltage <voltage>\n");
  }
  OkayToSend();
}
void abort()
{
  printf(Serial, "Abort!\n");
  motion.abort();
  //Also need to turn of torch and retract to clearance height!
}
void soft_abort()
{
  printf(Serial, "Soft!\n");
  motion.soft_abort();
  torch.cancel();
  torch.extinguish_torch();
}
void hold()
{
  printf(Serial, "Feedhold!\n");
  motion.feedhold();
}
void run()
{
  printf(Serial, "Run!\n");
  motion.run();
}
void invert_dir()
{
  char *axis = sCmd.next();
  char *set = sCmd.next();

  if (axis != NULL && set != NULL)
  {
    int axis_number = atoi(axis);
    int value = atoi(set);
    printf(Serial, "Setting axis %d invert to %d\n", axis_number, value);
    motion.invert_joint_dir(axis_number, value);
    torch.invert_joint_dir(axis_number, value);
  }
  else
  {
    printf(Serial, "Command usage: invert_dir <axis> <set>\n");
  }
}
void set_scale()
{
  char *axis = sCmd.next();
  char *set = sCmd.next();

  if (axis != NULL && set != NULL)
  {
    int axis_number = atoi(axis);
    double value = atof(set);
    printf(Serial, "Setting axis %d scale to %.4f\n", axis_number, value);
    motion.set_axis_scale(axis_number, value); //Axes that are out of range are ignored
    torch.set_axis_scale(axis_number, value); //Axes that are out of range are ignored
  }
  else
  {
    printf(Serial, "Command usage: set_scale <axis> <scale>\n");
  }
}
void set_torch()
{
  char *rapid = sCmd.next();
  char *probe = sCmd.next();
  char *takeup = sCmd.next();

  if (rapid != NULL && probe != NULL && takeup != NULL)
  {
    syncConfig.z_rapid_feed = atof(rapid) / 60;
    syncConfig.z_probe_feed  = atof(probe) / 60;
    syncConfig.floating_head_takeup = atof(takeup);
    printf(Serial, "Setting torch parameters\n\tRapid Feed: %.4f\n\tProbe Feed: %.4f\n\tFloating head takeup: %.4f\n", syncConfig.z_rapid_feed, syncConfig.z_probe_feed, syncConfig.floating_head_takeup);
  }
  else
  {
    printf(Serial, "Command usage: set_torch <rapid IPM> <probe IPM> <floating head takeup in units>\n");
  }
}
void set_thc_config()
{
  char *pin = sCmd.next();
  char *filtering = sCmd.next();
  char *velocity = sCmd.next();

  if (pin != NULL && filtering != NULL && velocity != NULL)
  {
    if (pin == "A19")
    {
      torch.set_thc_pin(A19);
      printf(Serial, "Setting thc pin to: %s\n", pin);
    }
    else if (pin == "A20")
    {
      torch.set_thc_pin(A20);
      printf(Serial, "Setting thc pin to: %s\n", pin);
    }
    else if (pin == "A21")
    {
      torch.set_thc_pin(A21);
      printf(Serial, "Setting thc pin to: %s\n", pin);
    }
    else
    {
      printf(Serial, "Unknown Pin!\n");
    }
    if (atoi(filtering) > 0 && atoi(filtering) < 20000)
    {
      torch.set_thc_filter(atoi(filtering));
      printf(Serial, "Setting thc filter to: %d\n", atoi(filtering));
    }
    else
    {
      printf(Serial, "thc filter must be between 0 and 20,000\n");
    }
    
    torch.set_thc_velocity(atof(velocity) / 60.0);
    printf(Serial, "Setting thc comp velocity to %.4f inches/min (%.4f inches/sec)\n", atof(velocity), atof(velocity) / 60.0);
    
  }
  else
  {
    printf(Serial, "Command usage: set_thc_config <Analog Pin Name> <filter cycle between 0 and 20,000> <comp velocity in inches/min>\n");
  }
}
void position_report()
{
  XYZ_Double position = motion.get_current_position();
  printf(Serial, "CurrentPosition: X: %0.4f Y: %.4f\n", position.x, position.y);
}
void hello()
{
  printf(Serial, "Hello at: %d\n", millis());
}
void dump_moves()
{
  torch.dump_move();
  printf(Serial, "*************** Current move ******************\n");
  motion.dump_current_move_to_serial();
  
  int number_of_moves = MoveStack->numElements(MoveStack);
  printf(Serial, "There are %d move/s currently on the stack\n", number_of_moves);
  for (int x = 0; x < number_of_moves; x++)
  {
    noInterrupts();
    if (MoveStack->peek(MoveStack, x) != NULL) //This element exists
    {
      struct Move_Data *move = (Move_Data*)MoveStack->peek(MoveStack, x);
      if (move->move_type == RAPID_MOVE)
      {
        printf(Serial, "RAPID-> X: %ld Y: %ld F: %ld\n", move->target.x, move->target.y, move->target.f);
      }
      if (move->move_type == LINE_MOVE)
      {
        printf(Serial, "LINE-> X: %ld Y: %ld F: %ld\n", move->target.x, move->target.y, move->target.f);
      }
      if (move->move_type == SYNC_MOVE)
      {
        printf(Serial, "SYNC_MOVE\n");
      }
    }
    interrupts();
  }
  printf(Serial, "End of moves!\n");
}
void init()
{
  motion.init();
  printf(Serial, "ok\n");
}
void movez()
{
  char *distance = sCmd.next();
  char *feedrate = sCmd.next();

  if (distance != NULL && feedrate != NULL)
  {
    double feed = atof(feedrate) / 60;
    double dist = atof(distance);
    printf(Serial, "Moving Z %.4f units at %.4f units/min\n", dist, feed * 60);
    torch.move_z_incremental(dist, feed, NULL, NULL);
  }
  else
  {
    printf(Serial, "Command usage: movez <distance> <feedrate>\n");
  }
}
void probe_z()
{
  callback.pierceHeight = 0.250;
  callback.pierceDelay = 1.5;
  callback.cutHeight = 0.250;
  motion.push_sync(&probe_torch_and_finish);
  OkayToSend();
}
/* Begin Gcode functions after here */
void rapid_move()
{
  char *first_word, *second_word;

  //Need to default polulate this with the current MCS position once motion library is up and running
  XYZ_Double current = motion.get_last_moves_target();
  double x = current.x;
  double y = current.y;

  first_word = sCmd.next();
  if (first_word != NULL)
  {
    if (first_word[0] == 'X')
    {
      first_word[0] = ' ';
      x = atof(first_word);
    }
    else if (first_word[0] == 'Y')
    {
      first_word[0] = ' ';
      y = atof(first_word);
    }
  }
  second_word = sCmd.next();
  if (second_word != NULL)
  {
    if (second_word[0] == 'X')
    {
      second_word[0] = ' ';
      x = atof(second_word);
    }
    else if (second_word[0] == 'Y')
    {
      second_word[0] = ' ';
      y = atof(second_word);
    }
  }
  printf(Serial, "Rapid move to X%.4f Y%.4f\n", x, y);
  XYZ_Double t;
  t.x = x;
  t.y = y;
  t.z = 0;
  t.f = 600.0; //Rapid feedrate, units/min
  motion.push_target(t, RAPID_MOVE);
  OkayToSend();
}
void line_move()
{
  char *first_word, *second_word, *third_word;

  //Need to default polulate this with the current MCS position once motion library is up and running
  XYZ_Double current = motion.get_last_moves_target();
  double x = current.x;
  double y = current.y;
  double f = current.f;

  first_word = sCmd.next();
  if (first_word != NULL)
  {
    if (first_word[0] == 'X')
    {
      first_word[0] = ' ';
      x = atof(first_word);
    }
    else if (first_word[0] == 'Y')
    {
      first_word[0] = ' ';
      y = atof(first_word);
    }
    else if (first_word[0] == 'F')
    {
      first_word[0] = ' ';
      f = atof(first_word);
    }
  }
  second_word = sCmd.next();
  if (second_word != NULL)
  {
    if (second_word[0] == 'X')
    {
      second_word[0] = ' ';
      x = atof(second_word);
    }
    else if (second_word[0] == 'Y')
    {
      second_word[0] = ' ';
      y = atof(second_word);
    }
    else if (first_word[0] == 'F')
    {
      second_word[0] = ' ';
      f = atof(first_word);
    }
  }
  third_word = sCmd.next();
  if (third_word != NULL)
  {
    if (third_word[0] == 'X')
    {
      third_word[0] = ' ';
      x = atof(third_word);
    }
    else if (third_word[0] == 'Y')
    {
      third_word[0] = ' ';
      y = atof(third_word);
    }
    else if (third_word[0] == 'F')
    {
      third_word[0] = ' ';
      f = atof(third_word);
    }
  }
  printf(Serial, "line move to X%.4f Y%.4f F%.4f\n", x, y, f);
  XYZ_Double t;
  t.x = x;
  t.y = y;
  t.z = 0;
  t.f = f; //Rapid feedrate, units/min
  motion.push_target(t, LINE_MOVE);
  OkayToSend();
}

/* End Gcode functions before here */
void gcodes_init()
{
  Serial.begin(9600);

  //All special commands below here
  sCmd.addCommand("init", init);
  sCmd.addCommand("hello", hello);
  sCmd.addCommand("dump_moves", dump_moves);

  //Real-Time commands, these commands should not return an OK!
  sCmd.addCommand("?", position_report);
  sCmd.addCommand("hold", hold);
  sCmd.addCommand("run", run);
  sCmd.addCommand("abort", abort);
  sCmd.addCommand("soft_abort", soft_abort);
  sCmd.addCommand("movez", movez);
  sCmd.addCommand("probe_z", probe_z);
  sCmd.addCommand("invert_dir", invert_dir);
  sCmd.addCommand("set_scale", set_scale);
  sCmd.addCommand("set_torch", set_torch);
  sCmd.addCommand("set_thc_config", set_thc_config);


  //All Gcode commands below here
  sCmd.addCommand("G0", rapid_move);
  sCmd.addCommand("G1", line_move);
  sCmd.addCommand("fire_torch", fire_torch);
  sCmd.addCommand("torch_off", torch_off);
  sCmd.addCommand("set_voltage", set_voltage);

  sCmd.setDefaultHandler(unrecognized);

  PendingOkay = false;
}
void gcodes_tick()
{
  if (PendingOkay == true && MoveStack->isFull(MoveStack) == false)
  {
    PendingOkay = false;
    printf(Serial, "ok: %d\n", MoveStack->numElements(MoveStack)); //Send an okay once there is room in the stack!
  }
  sCmd.readSerial();
}
void OkayToSend()
{
  if (sCmd.CRCpassed())
  {
    PendingOkay = true;
  }
}