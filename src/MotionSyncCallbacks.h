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
};
extern CallbackData callback;
/**********************
 * GLOBAL PROTOTYPES
 **********************/

/* Condition met callbacks */
void probe_torch();
void retract_torch();
void fire_torch();
void light_torch_and_pierce_delay();


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
