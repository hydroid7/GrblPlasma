#define LED 13
#define MIN_FEED_RATE 0.016666

/* 
    Note that Rev 1 had some board mistakes that will require the next revision to have pin changes.
    We need to implement some ifdef system that will change the pinmap based on what revision is defined at compile time
*/
#define ARC_START_PIN 28
#define ARC_OK_PIN 14
#define ARC_VOLTAGE_PIN A20

#define X_ENABLE_PIN 6
#define X_STEP_PIN 0
#define X_DIR_PIN 1

#define Y1_ENABLE_PIN 7
#define Y1_STEP_PIN 2
#define Y1_DIR_PIN 3


#define Y2_ENABLE_PIN 7
#define Y2_STEP_PIN 9
#define Y2_DIR_PIN  10

#define Z_ENABLE_PIN 8
#define Z_STEP_PIN 4
#define Z_DIR_PIN 5
#define Z_PROBE_PIN 27
#define Z_PROBE_FEEDRATE 1.5 //Set the feedrate to 10 inches per minute
#define Z_RAPID_FEEDRATE 2
#define Z_FLOATING_HEAD_TAKEUP 0.2