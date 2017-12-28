#ifndef MC_PROCESS_H_
#define MC_PROCESS_H_
/*===============================================================================================*/
/**
 *   @file mc_process.h
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
#include "target.h"
#include "command.h"
#include "timers.h"

/* Scheduler include files. */

/* Application include files. */

/*=================================================================================================
 CONSTANTS
=================================================================================================*/


/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
extern TimerHandle_t xmcPwrKeyCheckTimer;

void sendTaskStartSignal();
void sendSTOPandPowerOFF(void);
void check_pwronstate();
boolean check_pwroffstate(void);
void PlasmaVolSend2Task(command_type *cmd);


/*===============================================================================================*/
#endif  /* MC_PROCESS_H_ */
