#ifndef _PLASMA_STATE_H_
#define _PLASMA_STATE_H_
/*===============================================================================================*/
/**
 *   @file Plasma_state.h
 *
 *   @version v1.0
 */
/*=================================================================================================


Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------


===================================================================================================*/
/* Standard includes. */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Application include files. */
#include "command.h"


/*=================================================================================================
 CONSTANTS
=================================================================================================*/



/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
void PlasmaState_init(TimerCallbackFunction_t pxCallbackFunction);
boolean CheckIsPlasmaCmd(command_type *command);


/*===============================================================================================*/
#endif  /* _PLASMA_STATE_H_ */
