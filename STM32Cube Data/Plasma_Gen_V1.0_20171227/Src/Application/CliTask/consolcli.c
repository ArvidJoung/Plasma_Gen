/*
    FreeRTOS V8.2.0rc1 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?".  Have you defined configASSERT()?  *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *   Investing in training allows your team to be as productive as       *
     *   possible as early as possible, lowering your overall development    *
     *   cost, and enabling you to bring a more robust product to market     *
     *   earlier than would otherwise be possible.  Richard Barry is both    *
     *   the architect and key author of FreeRTOS, and so also the world's   *
     *   leading authority on what is the world's most popular real time     *
     *   kernel for deeply embedded MCU designs.  Obtaining your training    *
     *   from Richard ensures your team will gain directly from his in-depth *
     *   product knowledge and years of usage experience.  Contact Real Time *
     *   Engineers Ltd to enquire about the FreeRTOS Masterclass, presented  *
     *   by Richard Barry:  http://www.FreeRTOS.org/contact
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    You are receiving this top quality software for free.  Please play *
     *    fair and reciprocate by reporting any suspected issues and         *
     *    participating in the community forum:                              *
     *    http://www.FreeRTOS.org/support                                    *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/**
 * Creates eight tasks, each of which flash an LED at a different rate.  The first 
 * LED flashes every 125ms, the second every 250ms, the third every 375ms, etc.
 *
 * The LED flash tasks provide instant visual feedback.  They show that the scheduler 
 * is still operational.
 *
 * The PC port uses the standard parallel port for outputs, the Flashlite 186 port 
 * uses IO port F.
 *
 * \page flashC flash.c
 * \ingroup DemoFiles
 * <HR>
 */

/*
Changes from V2.0.0

	+ Delay periods are now specified using variables and constants of
	  TickType_t rather than unsigned long.

Changes from V2.1.1

	+ The stack size now uses configMINIMAL_STACK_SIZE.
	+ String constants made file scope to decrease stack depth on 8051 port.
*/

#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "target.h"
#include "consol.h"
#include "consolcli.h"
#include "debugmsgcli.h"
#include "task_cfg.h"
#include "FreeRTOS_CLI.h"

#if defined(DEV_VERSION)
char debugMsgDispFlag = 1;
#else
char debugMsgDispFlag = 0;
#endif
/*-----------------------------------------------------------*/

portBASE_TYPE prvCONSOLCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
portBASE_TYPE prvCONSOLGenCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
portBASE_TYPE prvCONSOLExpCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
const CLI_Command_Definition_t xDebugMsgDispStats =
{
	"debug",			/* The command string to type. */
	"\r\ndebug on/off/low/med/hi/err/none/item :\r\n Debug message displays on/off/low \r\n Set Global Debug Level to /med/hi/err/none.\r\n",
	prvCONSOLCommand,	/* The function to run. */
	-1					/* 1 parameters are expected. */
};

const CLI_Command_Definition_t xDebugGENDispStats =
{
	"gen",			/* The command string to type. */
	"\r\ngen low/med/hi/err/none :\r\n Set GEN Debug message state.\r\n",
	prvCONSOLGenCommand,	/* The function to run. */
	-1					/* Variable parameters are expected. */
};

const CLI_Command_Definition_t xDebugEXPDispStats =
{
	"exp",			/* The command string to type. */
	"\r\nexp low/med/hi/err/none :\r\n Set EXP Debug message state.\r\n",
	prvCONSOLExpCommand,	/* The function to run. */
	-1					/* Variable parameters are expected. */
};

portBASE_TYPE prvCONSOLCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader1 = "\r\nDebug message display are Enabled.\r\n ";
	const char *const pcHeader2 = "\r\nDebug message display are Disabled.\r\n ";
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
							1,						/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);


	if( pcParameter != NULL)
	{
		if( strncmp(pcParameter,"off",3) == 0) 
		{
			debugMsgDispFlag=0;
			strcpy( pcWriteBuffer, pcHeader2 );
		}
		else if( strncmp(pcParameter,"on",2) == 0) 
		{
			debugMsgDispFlag=1;
			strcpy( pcWriteBuffer, pcHeader1 );
		}
		else if( strncmp(pcParameter,"low",3) == 0) 
		{
			debugSetLevels(DEBUGMSGLOW);
		}
		else if( strncmp(pcParameter,"med",3) == 0) 
		{
			debugSetLevels(DEBUGMSGMED);
		}
		else if( strncmp(pcParameter,"hi",2) == 0) 
		{
			debugSetLevels(DEBUGMSGHI);
		}
		else if( strncmp(pcParameter,"err",3) == 0) 
		{
			debugSetLevels(DEBUGMSGERR);
		}
		else if( strncmp(pcParameter,"none",4) == 0) 
		{
			debugSetLevels(DEBUGMSGNON);
		} 
		else if( strncmp(pcParameter,"item",4) == 0) 
		{
			pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2 ,&xParameterStringLength);
            if( pcParameter != NULL)
            {
			    debugSetTestItem((uint8_t)(atoi((const char *)pcParameter)));
            }
            else
            {
                const char *const debugHelpItemStr = "\r\n Debug item no \r\n";
                strncpy( pcWriteBuffer, debugHelpItemStr, strlen(debugHelpItemStr) );
            }
		}        
        else
        {
            strncpy( pcWriteBuffer, xDebugMsgDispStats.pcHelpString, strlen(xDebugMsgDispStats.pcHelpString) );
        }
	}
    else
    {
        if(debugMsgDispFlag)
        {
            strcpy( pcWriteBuffer, pcHeader1 );
        }
        else
        {
            strcpy( pcWriteBuffer, pcHeader2 );
        }
    }

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}


portBASE_TYPE prvCONSOLGenCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "\r\nGEN Debug display set to ";
	const char *pcParameter;
	portBASE_TYPE xParameterStringLength;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							1,						/* uxWantedParameter: Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);


	if( pcParameter == NULL)
	{
		char *tt;

		strncpy( pcWriteBuffer, xDebugGENDispStats.pcHelpString, xWriteBufferLen );
		tt = debugGetCurLevelStr(DEBUGGENMSG);
		strcat( pcWriteBuffer, "GEN " );
		strcat( pcWriteBuffer, tt );
		strcat( pcWriteBuffer, "\r\n" );
	}
	else
	{
		if( strncmp(pcParameter,"low",3) == 0) 
		{
			debugSetCurLevel(DEBUGGENMSG,DEBUGMSGLOW);
		}
		else if( strncmp(pcParameter,"med",3) == 0) 
		{
			debugSetCurLevel(DEBUGGENMSG,DEBUGMSGMED);
		}
		else if( strncmp(pcParameter,"hi",2) == 0) 
		{
			debugSetCurLevel(DEBUGGENMSG,DEBUGMSGHI);
		}
		else if( strncmp(pcParameter,"err",3) == 0) 
		{
			debugSetCurLevel(DEBUGGENMSG,DEBUGMSGERR);
		}
		else if( strncmp(pcParameter,"none",4) == 0) 
		{
			debugSetCurLevel(DEBUGGENMSG,DEBUGMSGNON);
		}
		strcpy( pcWriteBuffer, pcHeader );
		strcat( pcWriteBuffer, pcParameter );
		strcat( pcWriteBuffer, "\r\n" );
	}
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

portBASE_TYPE prvCONSOLExpCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "\r\nEXP Debug display set to ";
	const char *pcParameter;
	portBASE_TYPE xParameterStringLength;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							1,						/* uxWantedParameter: Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);


	if( pcParameter == NULL)
	{
		char *tt;

		strncpy( pcWriteBuffer, xDebugGENDispStats.pcHelpString, xWriteBufferLen );
		tt = debugGetCurLevelStr(DEBUGEXPMSG);
		strcat( pcWriteBuffer, "EXP " );
		strcat( pcWriteBuffer, tt );
		strcat( pcWriteBuffer, "\r\n" );
	}
	else
	{
		if( strncmp(pcParameter,"low",3) == 0) 
		{
			debugSetCurLevel(DEBUGEXPMSG,DEBUGMSGLOW);
		}
		else if( strncmp(pcParameter,"med",3) == 0) 
		{
			debugSetCurLevel(DEBUGEXPMSG,DEBUGMSGMED);
		}
		else if( strncmp(pcParameter,"hi",2) == 0) 
		{
			debugSetCurLevel(DEBUGEXPMSG,DEBUGMSGHI);
		}
		else if( strncmp(pcParameter,"err",3) == 0) 
		{
			debugSetCurLevel(DEBUGEXPMSG,DEBUGMSGERR);
		}
		else if( strncmp(pcParameter,"none",4) == 0) 
		{
			debugSetCurLevel(DEBUGEXPMSG,DEBUGMSGNON);
		}
		strcpy( pcWriteBuffer, pcHeader );
		strcat( pcWriteBuffer, pcParameter );
		strcat( pcWriteBuffer, "\r\n" );
	}
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

void consolDebugCLIregister(void)
{
	/* Register debug command line commands defined above. */
	FreeRTOS_CLIRegisterCommand( &xDebugMsgDispStats );
	FreeRTOS_CLIRegisterCommand( &xDebugGENDispStats );
	FreeRTOS_CLIRegisterCommand( &xDebugEXPDispStats );
}
