#ifndef TASK_CMDS_H_
#define TASK_CMDS_H_
/*===============================================================================================*/
/**
 *   @file task_cmds.h
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

/* Scheduler include files. */

/* Application include files. */

/*=================================================================================================
 CONSTANTS
=================================================================================================*/


/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
void sendMcCommand(uint16_t command);
void sendtoMcCmdConstBuf(uint16_t command, void * data, uint16_t len);
void sendtoInEventCmd(uint16_t command);
void sendtoOutEventCmd(uint16_t command);
void sendtoBuzzerOnCmd(uint8_t type);

/*===============================================================================================*/
#endif  /* TASK_CMDS_H_ */
