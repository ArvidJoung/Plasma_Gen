/*===============================================================================================*/
/**
 *   @file debugmsgcli.c
 *
 *   @version v1.0
 */
/*=================================================================================================

 (c) Copyright , All Rights Reserved

Revision History:

Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------
jsyoon          2015/01/05

=================================================================================================*/

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
//#include "Stm32f4xx_hal.h"

/* Application includes. */
#include "comdef.h"
#include "debugmsgcli.h"
#include "consolcli.h"


/* debug msg buffer */
static char debugMsgBuffer[ DEBUGMSGLENMAX + 1 ];

const char const *debugMsgLevelStr[] = 
{
	"Debug Level ALL",
	"Debug Level LOW",
	"Debug Level MED",
	"Debug Level HIGH",
	"Debug Level ERROR",
	"Debug Level NONE"
};


/* default debug module & msg level */

static uint32 debugMsgLevel = 
		( (DEBUGMSGHI * MCMSGAREA) |
		  (DEBUGMSGHI * GENMSGAREA) | 
		  (DEBUGMSGHI * OUTEVENTMSGAREA) |
		  (DEBUGMSGHI * INEVENTMSGAREA) |
          (DEBUGMSGNON * EXPMSGAREA)) ;

static uint8 debugTestItemVal = 0;

/*-----------------------------------------------------------*/


#define DISPLAYCOLOR

#ifdef DISPLAYCOLOR
static uint8 debugGetLevelColor(const char *level)
{
	char color = level[0];
	uint8 retVal = 0;

    switch(color)
    {
	case 'L': 
	default:
		retVal = 0; break;
	case 'H': retVal = 2; break;
	case 'E': retVal = 3; break;
	}

	return retVal;
}

static void debugPrintLevelColor(uint8 level)
{
    switch(level)
    {
        case 3:
            //vDisplayMessage("\033[1;31m");  // bold red
            vDisplayMessage("\033[0;35m");    // Magenta
            break;
        case 2:
            //vDisplayMessage("\033[34m");    //  blue
            vDisplayMessage("\033[33m");    //  yellow
            break;
        default:
            vDisplayMessage("\033[0m");     // default color
            break;
    }
}
#endif

uint8 GetShiftValue(uint32 moduleMsgMask)
{
	uint8 retVal = 0;

	switch(moduleMsgMask)
	{
	case DEBUGMCMSG:
	default:
		break;
	case DEBUGOUTEVENTMSG: retVal = 4;	break;
	case DEBUGINEVENTMSG: retVal = 8;	break;
	case DEBUGGENMSG: retVal = 12;	break;
	case DEBUGEXPMSG: retVal = 24;	break;
	}

	return retVal;
}

void debugvsPrint ( const char *fmt, ... )
{
    va_list args;
    int len=0;

    va_start ( args, fmt );
    len = vsnprintf ( debugMsgBuffer, DEBUGMSGLENMAX, fmt, args );
    va_end ( args );

    if ( len <= 0 )
        return;

    if ( len > DEBUGMSGLENMAX )
	{
		// debug buffer overflow!!!!!
	    len = MIN ( len, DEBUGMSGLENMAX );
	}
	debugMsgBuffer[ len ] = 0;   // add null character
	vDisplayMessage(debugMsgBuffer);
}

/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
uint8 debugGetCurLevel ( uint32 moduleMsgMask )
{
	uint32 levelVal = (debugMsgLevel & moduleMsgMask);

	levelVal = levelVal >> GetShiftValue(moduleMsgMask);

	return (uint8) levelVal;
}

char *debugGetCurLevelStr ( uint32 moduleMsgMask )
{
	char * pRet = NULL;

	switch( debugGetCurLevel(moduleMsgMask))
	{
	case DEBUGMSGALL: pRet = (char * )debugMsgLevelStr[DEBUGMSGALL]; break;
	case DEBUGMSGLOW: pRet = (char * )debugMsgLevelStr[DEBUGMSGLOW]; break;
	case DEBUGMSGMED: pRet = (char * )debugMsgLevelStr[DEBUGMSGMED]; break;
	case DEBUGMSGHI: pRet = (char * )debugMsgLevelStr[DEBUGMSGHI]; break;
	case DEBUGMSGERR: pRet = (char * )debugMsgLevelStr[DEBUGMSGERR]; break;
	default: break;
	}
	return pRet;
}

uint32 debugSetLevels ( uint8 level )
{
/*    debugMsgLevel = ( (level * MCMSGAREA) |
                      (level * LEDMSGAREA) | 
                      (level * BLEMSGAREA) | 
                      (level * GENMSGAREA) | 
                      (level * MP3MSGAREA) | 
                      (level * SENMSGAREA) |
                      (level * EXPMSGAREA)) ;
*/
    debugMsgLevel = debugMsgLevel & DEBUGEXPMSG;
    debugMsgLevel |= ( (level * MCMSGAREA) |
                      (level * OUTEVENTMSGAREA) |
                      (level * INEVENTMSGAREA) |
                      (level * GENMSGAREA)
					 );
	return debugMsgLevel;
}

uint32 debugSetCurLevel ( uint32 moduleMsgMask, uint8 level )
{
	uint32 levelVal = level;

	levelVal = levelVal << GetShiftValue(moduleMsgMask);
	debugMsgLevel &= ~moduleMsgMask;
	debugMsgLevel |= levelVal;

	return debugMsgLevel;
}


/*------------------TEST ITEM ----------------------------------*/
uint8 debugGetTestItem ( uint8_t testitem )
{
    return ( testitem == debugTestItemVal ? 1 : 0);
}

void debugSetTestItem ( uint8_t testitem )
{
    debugTestItemVal = testitem;
}
/*------------------TEST ITEM ----------------------------------*/

// debugging print
void debugPrint ( const char *level, const char *file, int line, const char *fmt, ... )
{
	if (debugMsgDispFlag)
	{
        va_list args;
        int len=0;

    	#ifdef SYS_HAD_RTC
            RTC_t       rtc;
        #elif defined(SYS_HAD_TICK)
            uint32_t    tick;
    	#endif

        const char *ptr;

        //portENTER_CRITICAL();
    	//vTaskSuspendAll();

    	// display system time information
        #ifdef SYS_HAD_RTC
            RTC_GetTime ( &rtc );
        #elif defined(SYS_HAD_TICK)
            tick = HAL_GetTick();
    	#endif

    	#ifdef DISPLAYCOLOR
            debugPrintLevelColor(debugGetLevelColor(level));
        #endif

    	#ifdef DISPLAYRTC
            debugvsPrint ( "%02d:%02d:%02d ", rtc.hour, rtc.min, rtc.sec );
        #elif defined(SYS_HAD_TICK)
            //uint32_t msec = tick%1000;
            uint32_t    totalsec    = tick/1000;
            uint8_t     sec         = totalsec % 60;
            uint32_t    totalmin    = totalsec / 60;
            uint8_t     min         = totalmin % 60;
            uint8_t     hour        = totalmin / 60;
            debugvsPrint ( "%02d:%02d:%02d ", hour, min, sec );
    	#endif

        // display file, line and level  information
        ptr = strrchr ( file, '\\' );
        ptr = ( ptr != NULL ) ? ( ptr + 1 ) : "----";

        debugvsPrint ( "%15s %4d %s ", ptr, line, ( ( level != NULL ) ? level : "-" ) );

        // display message
        va_start ( args, fmt );
        len = vsnprintf ( debugMsgBuffer, DEBUGMSGLENMAX, fmt, args );
        va_end ( args );

        if ( len <= 0 )
            return;

        if ( len > DEBUGMSGLENMAX )
    	{
    		// debug buffer overflow!!!!!
    	    len = MIN ( len, DEBUGMSGLENMAX );
    	}
    	debugMsgBuffer[ len ] = 0;   // add null character
    	vDisplayMessage(debugMsgBuffer);

    	#ifdef DISPLAYCOLOR
        debugPrintLevelColor(0);
    	#endif
    	//xTaskResumeAll();
        //portEXIT_CRITICAL();
	}
}

void debugTestPrint (uint8_t itemval, const char *fmt, ... )
{
	if (debugMsgDispFlag)
	{
        va_list args;
        int len=0;

        //portENTER_CRITICAL();
    	//vTaskSuspendAll();

        debugvsPrint ( "Item %02d:", itemval );

        // display message
        va_start ( args, fmt );
        len = vsnprintf ( debugMsgBuffer, DEBUGMSGLENMAX, fmt, args );
        va_end ( args );

        if ( len <= 0 )
            return;

        if ( len > DEBUGMSGLENMAX )
    	{
    		// debug buffer overflow!!!!!
    	    len = MIN ( len, DEBUGMSGLENMAX );
    	}
    	debugMsgBuffer[ len ] = 0;   // add null character
    	vDisplayMessage(debugMsgBuffer);

    	//xTaskResumeAll();
        //portEXIT_CRITICAL();
	}
}

void debugTimeDataPrint (const char *fmt, ... )
{
    if (debugMsgDispFlag)
    {
        va_list args;
        int len=0;

#ifdef SYS_HAD_RTC
        RTC_t       rtc;
#elif defined(SYS_HAD_TICK)
        uint32_t    tick;
#endif

        //portENTER_CRITICAL();
    	//vTaskSuspendAll();

    	// display system time information
#ifdef SYS_HAD_RTC
        RTC_GetTime ( &rtc );
#elif defined(SYS_HAD_TICK)
        tick = HAL_GetTick();
#endif

#ifdef DISPLAYRTC
        debugvsPrint ( "%02d:%02d:%02d ", rtc.hour, rtc.min, rtc.sec );
#elif defined(SYS_HAD_TICK)
        //uint32_t msec = tick%1000;
        uint32_t    totalsec    = tick/1000;
        uint8_t     sec         = totalsec % 60;
        uint32_t    totalmin    = totalsec / 60;
        uint8_t     min         = totalmin % 60;
        uint8_t     hour        = totalmin / 60;
        debugvsPrint ( "%02d:%02d:%02d ", hour, min, sec );
#endif

        // display message
        va_start ( args, fmt );
        len = vsnprintf ( debugMsgBuffer, DEBUGMSGLENMAX, fmt, args );
        va_end ( args );

        if ( len <= 0 )
            return;

        if ( len > DEBUGMSGLENMAX )
        {
            // debug buffer overflow!!!!!
            len = MIN ( len, DEBUGMSGLENMAX );
        }
        debugMsgBuffer[ len ] = 0;   // add null character
        vDisplayMessage(debugMsgBuffer);

        //xTaskResumeAll();
        //portEXIT_CRITICAL();
    }
}

void debugDataPrint (const char *fmt, ... )
{
    if (debugMsgDispFlag)
    {
        va_list args;
        int len=0;

        //portENTER_CRITICAL();
    	//vTaskSuspendAll();

        // display message
        va_start ( args, fmt );
        len = vsnprintf ( debugMsgBuffer, DEBUGMSGLENMAX, fmt, args );
        va_end ( args );

        if ( len <= 0 )
            return;

        if ( len > DEBUGMSGLENMAX )
        {
            // debug buffer overflow!!!!!
            len = MIN ( len, DEBUGMSGLENMAX );
        }
        debugMsgBuffer[ len ] = 0;   // add null character
        vDisplayMessage(debugMsgBuffer);

        //xTaskResumeAll();
        //portEXIT_CRITICAL();
    }
}

