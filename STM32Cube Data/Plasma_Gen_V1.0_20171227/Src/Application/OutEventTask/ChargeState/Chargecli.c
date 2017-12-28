/*
    1 tab == 4 spaces!
*/

/**
 */

/*

*/

#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "consol.h"
#include "Chargecli.h"
#include "debugmsgcli.h"
#include "task_cfg.h"
#include "FreeRTOS_CLI.h"
#include "command.h"
#include "signals.h"
#include "interface.h"

/*-----------------------------------------------------------*/

portBASE_TYPE prvChargeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
portBASE_TYPE prvplasmaledCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
portBASE_TYPE prvstatusledCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
static const CLI_Command_Definition_t xTaskStats =
{
	"charge",				/* The command string to type. */
	"\r\ncharge low/med/high/error/none:\r\n Set Charge Debug message state.\r\n",
	prvChargeStatsCommand,	/* The function to run. */
	-1					/* 1 parameters are expected. */
						/* -1 인 경우 가변 parameter */
};

const CLI_Command_Definition_t plasmaledCommands =
{
	"plasma",				/* The command string to type. */
	"\r\nplasma [on/off/bri] [r,g,b] [0~255]:\r\n Control plasma led on/off/brightness.\r\n",
	prvplasmaledCommand,	/* The function to run. */
	-1					/* 1 parameters are expected. */
						/* -1 인 경우 가변 parameter */
};

const CLI_Command_Definition_t statusledCommands =
{
	"statusled",				/* The command string to type. */
	"\r\nstatusled [on/off] [0,1,2]:\r\n Control status led (0:Red, 1:Green, 2:Blue) on/off.\r\n",
	prvstatusledCommand,	/* The function to run. */
	-1					/* 1 parameters are expected. */
						/* -1 인 경우 가변 parameter */
};

portBASE_TYPE prvChargeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "\r\nCharge Debug display set to ";
	const char *pcParameter;
	portBASE_TYPE xParameterStringLength;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */

	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							1,						/* 몇번째 parameter가 필요 한지?, Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

	if( pcParameter == NULL)
	{
		char *tt;

		strncpy( pcWriteBuffer, xTaskStats.pcHelpString, xWriteBufferLen );
		tt = debugGetCurLevelStr(DEBUGOUTEVENTMSG);
		strcat( pcWriteBuffer, "CHARGE " );
		strcat( pcWriteBuffer, tt );
		strcat( pcWriteBuffer, "\r\n" );
	}
	else
	{
		if( strncmp(pcParameter,"low",3) == 0) 
		{
			//
			// 만일 2 번째 parameter가 필요 한경우 
			// 	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
			//
			debugSetCurLevel(DEBUGOUTEVENTMSG,DEBUGMSGLOW);
		}
		else if( strncmp(pcParameter,"med",3) == 0) 
		{
			debugSetCurLevel(DEBUGOUTEVENTMSG,DEBUGMSGMED);
		}
		else if( strncmp(pcParameter,"hi",2) == 0) 
		{
			debugSetCurLevel(DEBUGOUTEVENTMSG,DEBUGMSGHI);
		}
		else if( strncmp(pcParameter,"err",3) == 0) 
		{
			debugSetCurLevel(DEBUGOUTEVENTMSG,DEBUGMSGERR);
		}
		else if( strncmp(pcParameter,"none",4) == 0) 
		{
			debugSetCurLevel(DEBUGOUTEVENTMSG,DEBUGMSGNON);
		}
		strcpy( pcWriteBuffer, pcHeader );
		strcat( pcWriteBuffer, pcParameter );
		strcat( pcWriteBuffer, "\r\n" );
	}
	/* There is no more data to return after this single string, so return	pdFALSE. */
	return pdFALSE;
}


portBASE_TYPE prvplasmaledCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *pcParameter;
	portBASE_TYPE xParameterStringLength;
    boolean iscommandvalid = FALSE;
    command_type    cmd;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */

	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,		    /* The command string itself. */
							               1,						/* 몇번째 parameter가 필요 한지?, Return the next parameter. */
							               &xParameterStringLength	/* Store the parameter string length. */
						                  );

	if( pcParameter != NULL)
	{
		if( strncmp(pcParameter,"on",2) == 0) 
		{
            pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
            
            if( pcParameter != NULL)
            {
                
				iscommandvalid = TRUE;
				portENTER_CRITICAL();
				cmd.len = 0;
				cmd.isconst = FALSE;
				cmd.cmd = PLASMA_LED_ON_F;
				cmd.msg = NULL;
				xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
				portEXIT_CRITICAL();
            }
		}
		else if( strncmp(pcParameter,"off",3) == 0) 
		{
	        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
            
            if( pcParameter != NULL)
            {

				iscommandvalid = TRUE;
				portENTER_CRITICAL();
				cmd.len = FDSIZ(charge_cmd_msg_field_type,Cmd);
				cmd.isconst = FALSE;
				cmd.cmd = PLASMA_LED_OFF_F;
				cmd.msg = NULL;
				xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
				portEXIT_CRITICAL();
            }	
        }  
	}        
    
    if(iscommandvalid == FALSE)
    {
        strncpy( pcWriteBuffer, plasmaledCommands.pcHelpString, xWriteBufferLen );
    }
 	/* There is no more data to return after this single string, so return	pdFALSE. */
	return pdFALSE;
}


portBASE_TYPE prvstatusledCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *pcParameter;
	portBASE_TYPE xParameterStringLength;
    boolean iscommandvalid = FALSE;
    uint8_t *data;
    command_type    cmd;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */

	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,		    /* The command string itself. */
							               1,						/* 몇번째 parameter가 필요 한지?, Return the next parameter. */
							               &xParameterStringLength	/* Store the parameter string length. */
						                  );

	if( pcParameter != NULL)
	{
		if( strncmp(pcParameter,"on",2) == 0) 
		{
            pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
            
            if( pcParameter != NULL)
            {
                iscommandvalid = TRUE;
                if((data = (uint8_t *)cmd_malloc(FDPOS(charge_cmd_msg_field_type,EndofData))) != NULL)
                {
                    portENTER_CRITICAL();
                    cmd.len = FDSIZ(charge_cmd_msg_field_type,Cmd);
                    cmd.isconst = FALSE;
                    cmd.cmd = CHARGE_STATUS_ON_F;
                    data[0] = (uint8_t)(pcParameter[0] - '0');
                    cmd.msg = data;
                    xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
                    portEXIT_CRITICAL();
                }
                
                else
                {
                    DBGERR(GEN,"MEM Alloc Error!!!....\r\n");
                }
            }
		}
		else if( strncmp(pcParameter,"off",3) == 0) 
		{
	        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
            
            if( pcParameter != NULL)
            {
                iscommandvalid = TRUE;
                if((data = (uint8_t *)cmd_malloc(FDPOS(charge_cmd_msg_field_type,EndofData))) != NULL)
                {
                    portENTER_CRITICAL();
                    cmd.len = FDSIZ(charge_cmd_msg_field_type,Cmd);
                    cmd.isconst = FALSE;
                    cmd.cmd = CHARGE_STATUS_OFF_F;
                    data[0] = (uint8_t)(pcParameter[0] - '0');
                    cmd.msg = data;
                    xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
                    portEXIT_CRITICAL();
                }
                
                else
                {
                    DBGERR(GEN,"MEM Alloc Error!!!....\r\n");
                }
            }	
        }  
	}        
    
    if(iscommandvalid == FALSE)
    {
        strncpy( pcWriteBuffer, statusledCommands.pcHelpString, xWriteBufferLen );
    }
 	/* There is no more data to return after this single string, so return	pdFALSE. */
	return pdFALSE;
}

void ChargeCLIregister(void)
{
	/* Register debug command line commands defined above. */
	FreeRTOS_CLIRegisterCommand( &xTaskStats );
	FreeRTOS_CLIRegisterCommand( &plasmaledCommands );
	FreeRTOS_CLIRegisterCommand( &statusledCommands );
}
