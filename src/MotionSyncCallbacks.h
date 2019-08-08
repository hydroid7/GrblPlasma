#ifndef MOTIONSYNC_H
#define MOTIONSYNC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "MotionPlanner.h"
#include "TorchControl.h"
/*********************
 *      DEFINES
 *********************/
 
/**********************
 *      GLOBALS
 **********************/
struct CallbackData {
  double pierceHeight;
  double pierceDelay;
  double cutHeight;
  double clearanceHeight;
};
extern CallbackData callback;
/**********************
 * GLOBAL PROTOTYPES
 **********************/

/* Condition met callbacks */
void probe_torch_and_finish();
void retract_torch_and_finish();
void probe_torch();
void retract_torch();
void fire_torch();
void light_torch_and_pierce_delay();
void torch_off_and_retract();


/* Condition check callbacks */
bool stop_on_probe_input();

/**********************
 * CONTROLS PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
