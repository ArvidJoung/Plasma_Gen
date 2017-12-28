/*===============================================================================================*/
/**
 *   @file task_cmds.c
 *
 *   @version v1.0
 */
/*=================================================================================================

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------

Portability:
Indicate if this module is portable to other compilers or
platforms. If not, indicate specific reasons why is it not portable.
*/

/*===============================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Standard includes. */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Application includes. */
#include "target.h"
#include "comdef.h"
#include "signals.h"
#include "command.h"
#include "task_cfg.h"
#include "interface.h"
#include "task_cmds.h"
#include "debugmsgcli.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
/*-----------------------------------------------------------*/
void sendMcCommand(uint16_t command)
{
    command_type    cmd;

    portENTER_CRITICAL();
    cmd.len = 0;
    cmd.msg = NULL;
    cmd.cmd = command;
    cmd.isconst = TRUE;
    xQueueSend(xMcTaskQueue,&cmd,taskNO_BLOCK);
    portEXIT_CRITICAL();
}

void sendtoMcCmdConstBuf(uint16_t command, void * data, uint16_t len)
{
	command_type	cmd;

    portENTER_CRITICAL();
	cmd.len = len;
	cmd.msg = (void*)data;
    cmd.isconst = TRUE;
	cmd.cmd = command;
	xQueueSend(xMcTaskQueue,&cmd,taskNO_BLOCK);
    portEXIT_CRITICAL();
}

void sendtoInEventCmd(uint16_t command)
{
	command_type	cmd;

    portENTER_CRITICAL();
	cmd.len = 0;
	cmd.msg = NULL;
    cmd.isconst = TRUE;
	cmd.cmd = command;
	xQueueSend(xInEventTaskQueue,&cmd,taskNO_BLOCK);
    portEXIT_CRITICAL();
}

void sendtoOutEventCmd(uint16_t command)
{
	command_type	cmd;

    portENTER_CRITICAL();
	cmd.len = 0;
	cmd.msg = NULL;
    cmd.isconst = TRUE;
	cmd.cmd = command;
	xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
    portEXIT_CRITICAL();
}

void sendtoBuzzerOnCmd(uint8_t type)
{
    uint8_t         *data;

    if((data = (uint8_t *)cmd_malloc(FDPOS(gen_cmd_msg_field_type,EndofData))) != NULL)
    {
        command_type            scmd;
        
        scmd.len = FDPOS(gen_cmd_msg_field_type,EndofData);
        scmd.isconst = FALSE;
        scmd.cmd = BUZZER_SOUND_ON_F;
    
        data[FDPOS(gen_cmd_msg_field_type,data1)] = type;
        scmd.msg = data;
    
        portENTER_CRITICAL();
        xQueueSend(xOutEventTaskQueue,&scmd,taskNO_BLOCK);
        portEXIT_CRITICAL();
    }
    else
    {
        DBGERR(GEN,"Memory Allocation Error!!!\r\n");
    }      
}


void sendtoBuzzerOffCmd()
{
	command_type	cmd;

    portENTER_CRITICAL();
	cmd.len = 0;
	cmd.msg = NULL;
    cmd.isconst = TRUE;
	cmd.cmd = BUZZER_SOUND_OFF_F;
	xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
    portEXIT_CRITICAL();
}

/*===============================================================================================*/
