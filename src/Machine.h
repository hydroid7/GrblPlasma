#define LED 13
#define MIN_FEED_RATE 0.016666

#define ARC_START_PIN 28
#define ARC_VOLTAGE_PIN A20

#define X_STEP_PIN 0
#define X_DIR_PIN 1

#define Y1_STEP_PIN 2
#define Y1_DIR_PIN 3

#define Y2_STEP_PIN 9
#define Y2_DIR_PIN  10

#define Z_STEP_PIN 4
#define Z_DIR_PIN 5
#define Z_PROBE_PIN 27
#define Z_PROBE_FEEDRATE (10 / 60) //Set the feedrate to 10 inches per minute
#define Z_RAPID_FEEDRATE (75 / 60)
#define Z_FLOATING_HEAD_TAKEUP 0.1