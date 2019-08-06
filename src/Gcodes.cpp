#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "TorchControl.h"
#include "MotionSyncCallbacks.h"
#include "RingBuf.h"
#include "Gcodes.h"

bool PendingOkay;
bool WaitForMotionSync;
void (*MotionSyncCallback)();

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

  SyncMotion(&probe_torch);
}
void torch_off()
{
  callback.clearanceHeight = 2;
  SyncMotion(&torch_off_and_retract);
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
  int number_of_moves = MoveStack->numElements(MoveStack);
  printf(Serial, "There are %d move/s currently on the stack\n", number_of_moves);
  for (int x = 0; x < number_of_moves; x++)
  {
    noInterrupts();
    if (MoveStack->peek(MoveStack, x) != NULL) //This element exists
    {
      struct Move_Data *move = (Move_Data*)MoveStack->peek(MoveStack, x);
      printf(Serial, "-> X: %ld Y: %ld F: %ld\n", move->target.x, move->target.y, move->target.f);
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
  

  //All Gcode commands below here
  sCmd.addCommand("G0", rapid_move);
  sCmd.addCommand("G1", line_move);
  sCmd.addCommand("fire_torch", fire_torch);
  sCmd.addCommand("torch_off", torch_off);
  sCmd.addCommand("set_voltage", set_voltage);

  sCmd.setDefaultHandler(unrecognized);

  PendingOkay = false;
  WaitForMotionSync = false;
}
void gcodes_tick()
{
  if (PendingOkay == true && MoveStack->isFull(MoveStack) == false)
  {
    if (WaitForMotionSync == false) //We are not waiting for motion to sync, keep sending lines
    {
      PendingOkay = false;
      printf(Serial, "ok: %d\n", MoveStack->numElements(MoveStack)); //Send an okay once there is room in the stack!
    }
  }
  if (WaitForMotionSync == true)
  {
    //printf(Serial, "Waiting for motion to sync!\n");
    //Stop sending lines until the MoveStack is empty and the machine is not in motion anymore
    if (motion.is_in_motion() == false && MoveStack->isEmpty(MoveStack))
    {
      WaitForMotionSync = false; //The machine is at the last position sent!
      //printf(Serial, "Motion is synced!\n");
      if (MotionSyncCallback != NULL)  MotionSyncCallback();
    }
  }
  sCmd.readSerial();
}
void SyncMotion(void (*callback)())
{
  MotionSyncCallback = callback;
  WaitForMotionSync = true;
  PendingOkay = false; //The last callback that fires should send the okay!
}
void OkayToSend()
{
  PendingOkay = true;
}