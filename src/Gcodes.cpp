#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "RingBuf.h"
#include "Gcodes.h"

bool PendingOkay;

SerialCommand sCmd;
MotionPlanner motion;

void unrecognized(const char *command)
{
  printf(Serial, "\"%s\" is not a supported command!\n", command);
  PendingOkay = true;
}
void hold()
{
  printf(Serial, "Feedhold!\n");
  motion.feedhold();
  PendingOkay = true;
}
void run()
{
  printf(Serial, "Run!\n");
  motion.run();
  PendingOkay = true;
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
  t.f = 350.0; //Rapid feedrate, units/min
  motion.push_target(t, RAPID_MOVE);
  PendingOkay = true;
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
  PendingOkay = true;
}

/* End Gcode functions before here */
void gcodes_init()
{
  Serial.begin(9600);

  //All special commands below here
  sCmd.addCommand("init", init);
  sCmd.addCommand("hello", hello);
  sCmd.addCommand("dump_moves", dump_moves);

  //Real-Time commands
  sCmd.addCommand("?", position_report);
  sCmd.addCommand("hold", hold);
  sCmd.addCommand("run", run);

  //All Gcode commands below here
  sCmd.addCommand("G0", rapid_move);
  sCmd.addCommand("G1", line_move);

  sCmd.setDefaultHandler(unrecognized);

  PendingOkay = false;
}
void gcodes_tick()
{
  if (PendingOkay == true && MoveStack->isFull(MoveStack) == false)
  {
    PendingOkay = false;
    printf(Serial, "ok\n"); //Send an okay once there is room in the stack!
  }
  sCmd.readSerial();
}
