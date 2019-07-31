#include "Arduino.h"
#include "Machine.h"
#include "SerialCommand.h"
#include "MotionPlanner.h"
#include "RingBuf.h"
#include "Gcodes.h"

SerialCommand sCmd;
MotionPlanner motion;

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
}
/* Begin Gcode functions after here */
void rapid_move()
{
  char *first_word, *second_word;

  //Need to default polulate this with the current MCS position once motion library is up and running
  double x = 0;
  double y = 0;

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
  t.f = 30.0; //Rapid feedrate, units/min
  motion.push_target(t);
}

/* End Gcode functions before here */
void gcodes_init()
{
  Serial.begin(9600);

  //All special command below here
  sCmd.addCommand("init", init);
  sCmd.addCommand("hello", hello);
  sCmd.addCommand("dump_moves", dump_moves);

  //All Gcode commands below here
  sCmd.addCommand("G0", rapid_move);
}
void gcodes_tick()
{
  sCmd.readSerial();
}
