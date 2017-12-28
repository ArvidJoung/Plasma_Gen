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
#include "cmsis_os.h"

/* Demo program include files. */
#include "consol.h"
#include "InEventcli.h"
#include "debugmsgcli.h"
#include "task_cfg.h"
#include "FreeRTOS_CLI.h"
#include "command.h"
#include "signals.h"
#include "interface.h"
//#include "tim.h"

/*-----------------------------------------------------------*/

portBASE_TYPE prvInEventStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );


/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
static const CLI_Command_Definition_t xTaskStats =
{
	"inevent",				/* The command string to type. */
	"\r\nInEvent Task low/med/high/error/none:\r\n Set InEvent Debug message state.\r\n",
	prvInEventStatsCommand,	/* The function to run. */
	-1					/* 1 parameters are expected. */
						/* -1 인 경우 가변 parameter */
};

portBASE_TYPE prvInEventStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "\r\nInEvent Debug display set to ";
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
		tt = debugGetCurLevelStr(DEBUGINEVENTMSG);
		strcat( pcWriteBuffer, "INEVENT " );
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
			debugSetCurLevel(DEBUGINEVENTMSG,DEBUGMSGLOW);
		}
		else if( strncmp(pcParameter,"med",3) == 0) 
		{
			debugSetCurLevel(DEBUGINEVENTMSG,DEBUGMSGMED);
		}
		else if( strncmp(pcParameter,"hi",2) == 0) 
		{
			debugSetCurLevel(DEBUGINEVENTMSG,DEBUGMSGHI);
		}
		else if( strncmp(pcParameter,"err",3) == 0) 
		{
			debugSetCurLevel(DEBUGINEVENTMSG,DEBUGMSGERR);
		}
		else if( strncmp(pcParameter,"none",4) == 0) 
		{
			debugSetCurLevel(DEBUGINEVENTMSG,DEBUGMSGNON);
		}
		strcpy( pcWriteBuffer, pcHeader );
		strcat( pcWriteBuffer, pcParameter );
		strcat( pcWriteBuffer, "\r\n" );
	}
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}


void InEventCLIregister(void)
{
	/* Register debug command line commands defined above. */
	FreeRTOS_CLIRegisterCommand( &xTaskStats );
}
