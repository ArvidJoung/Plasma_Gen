#ifndef DEBUGMSGCLI_H
#define DEBUGMSGCLI_H
/*===============================================================================================*/
/**
 *   @file debugmsgcli.h
 *
 *   @version v1.0
 */
/*=================================================================================================

 (c) Copyright , All Rights Reserved

Revision History:

Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------
jsyoon          2015/01/05
===================================================================================================*/


/* Standard includes. */
/* FreeRTOS kernel includes. */
/* Application includes. */
#include "comdef.h"
#include "target.h"

/* CONSTANTS */

#define DEBUGMSGLENMAX		128


/* Message levels */
#define DEBUGMSGALL         0x0       /* All message printout     */
#define DEBUGMSGLOW         0x1       /* Low level message printout */
#define DEBUGMSGMED         0x2       /* Medium level message printout */
#define DEBUGMSGHI          0x3       /* High level message printout   */
#define DEBUGMSGERR         0x4       /* Error level message printout     */
#define DEBUGMSGNON         0x5       /* No messages              */
#define DEBUGMSGMASK        0xF

#define DEBUGMSGLOWSTR      "L"
#define DEBUGMSGMEDSTR      "M"
#define DEBUGMSGHISTR       "H"
#define DEBUGMSGERRSTR      "E"

/* Message module mask */
#define DEBUGMCMSG          0x0000000F      /* mc task message mask  */
#define DEBUGOUTEVENTMSG    0x000000F0      /* OutEvent task message mask  */
#define DEBUGINEVENTMSG     0x00000F00      /* InEvent task message mask  */
#define DEBUGGENMSG         0x0000F000      /*   */
#define DEBUGEXPMSG         0x0F000000      /*   */

#define MCMSGAREA           (0x00000001)     
#define OUTEVENTMSGAREA		(0x00000010)
#define INEVENTMSGAREA		(0x00000100)
#define GENMSGAREA			(0x00001000)  
#define EXPMSGAREA			(0x01000000)
/* debug message macro */

#ifndef USING_DBG_PRINT
#define DBGLOW(mask, ...)

#define DBGMED(mask, ...)

#define DBGHI(mask, ...)

#define DBGERR(mask, ...)

#else

#define DBGLOW(mask, ...)									\
	if ( DEBUGMSGLOW >= debugGetCurLevel(DEBUG##mask##MSG) ) \
        debugPrint( DEBUGMSGLOWSTR, __FILE__, __LINE__, __VA_ARGS__)

#define DBGMED(mask, ...)     \
	if ( DEBUGMSGMED >= debugGetCurLevel(DEBUG##mask##MSG) ) \
        debugPrint( DEBUGMSGMEDSTR, __FILE__, __LINE__, __VA_ARGS__)

#define DBGHI(mask, ...)    \
	if ( DEBUGMSGHI >= debugGetCurLevel(DEBUG##mask##MSG) ) \
        debugPrint( DEBUGMSGHISTR, __FILE__, __LINE__, __VA_ARGS__)

#define DBGERR(mask, ...)  \
	if ( DEBUGMSGERR >= debugGetCurLevel(DEBUG##mask##MSG) ) \
        debugPrint( DEBUGMSGERRSTR, __FILE__, __LINE__, __VA_ARGS__)
#endif

#define DBGTEST(val, ...)  \
	if ( debugGetTestItem(val) != 0 ) \
        debugTestPrint( val,__VA_ARGS__)

#define DBGTDATA(val, ...)  \
    if ( debugGetTestItem(val) != 0 ) \
        debugTimeDataPrint(__VA_ARGS__)
        
#define DBGDATA(val, ...)  \
    if ( debugGetTestItem(val) != 0 ) \
        debugDataPrint(__VA_ARGS__)

#if(0)
#define MSGTEST(msg, ...)  \
        debugTestPrint(1,"%s,%d :" msg "\n", __FILE__, __LINE__, ##__VA_ARGS__)

//#define MSGTEST(msg, ...)  debugTestPrint(1,"%15s,%4d :" msg "\n", getFilenameOnly(__FILE__), __LINE__, ##__VA_ARGS__)
//const char * getFilenameOnly( const char *pathNfilename)
//{
//    const char *ptr;
//    
//    // display file, line and level  information
//    ptr = strrchr ( pathNfilename, '\\' );
//    ptr = ( ptr != NULL ) ? ( ptr + 1 ) : "----";
//    return ptr;
//}

#endif

/* FUNCTION PROTOTYPES */

uint32 debugSetLevels ( uint8 level );
uint8 debugGetCurLevel ( uint32 moduleMsgMask );
uint32 debugSetCurLevel ( uint32 moduleMsgMask, uint8 level );
char *debugGetCurLevelStr ( uint32 moduleMsgMask );

uint8 debugGetTestItem ( uint8_t testitem );
void debugSetTestItem ( uint8_t testitem );

void debugPrint ( const char *level, const char *file, int line, const char *fmt, ... );
void debugTestPrint (uint8_t itemval, const char *fmt, ... );
void debugTimeDataPrint (const char *fmt, ... );
void debugDataPrint (const char *fmt, ... );

/*===============================================================================================*/
#endif 
