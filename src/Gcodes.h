#ifndef GCODES_H
#define GCODES_H

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
 #define _PRINTF_BUFFER_LENGTH_  500

 #if 1
 static char _pf_buffer_[_PRINTF_BUFFER_LENGTH_];
 #else
 extern char _pf_buffer_[_PRINTF_BUFFER_LENGTH_];
 #endif

 #define printf(_obj_,a,...)                                                   \
   do{                                                                   \
     snprintf(_pf_buffer_, sizeof(_pf_buffer_), a, ##__VA_ARGS__);       \
     _obj_.print(_pf_buffer_);                                    \
   }while(0)

 #define printfn(_obj_,a,...)                                                  \
   do{                                                                   \
     snprintf(_pf_buffer_, sizeof(_pf_buffer_), a"\r\n", ##__VA_ARGS__); \
     _obj_.print(_pf_buffer_);                                    \
   }while(0)
/**********************
 *      GLOBALS
 **********************/
extern MotionPlanner motion;
extern TorchControl torch;
extern bool OkayFlag;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
void gcodes_init();
void gcodes_tick();

void gcodes_cancel_sync_callback();

void SyncMotion(void (*callback)());
void OkayToSend();

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
