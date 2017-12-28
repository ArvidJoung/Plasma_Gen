#ifndef SIGNALS_H
#define SIGNALS_H
/*===============================================================================================*/
/**
 *   @file signals.h
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

*****************************************************
*****************************************************
    SIGNAL GROUP

    1. RESERVED     : 0x0000 ~
    2. XXXX TASK    : 0xX000 ~
    3. TIMER        : 0xX000 ~ 0xX0FF
    4. EVENT        : 0xX100 ~ 0xX1FF
    5. COMMAND      : 0xX200 ~ 0xX2FF

*****************************************************
1) MC TASK
    MC TASK         : 0x1000 ~
    TIMER           : 0x1000 ~ 0x10FF
    EVENT           : 0x1100 ~ 0x11FF
    COMMAND         : 0x1200 ~ 0x12FF

1. INEVENT TASK  : 0x2000 ~
    2. TIMER        : 0x2000 ~ 0x20FF
    3. EVENT        : 0x2100 ~ 0x21FF
    4. COMMAND      : 0x2200 ~ 0x22FF
    5. RESPONSE     : 0x2300 ~ 0x23FF

1. OUTEVENT TASK  : 0x3000 ~
    2. TIMER        : 0x3000 ~ 0x30FF
    3. EVENT        : 0x3100 ~ 0x31FF
    4. COMMAND      : 0x3200 ~ 0x32FF
    5. RESPONSE     : 0x3300 ~ 0x33FF

    1. CHARGE STATE : 0x3A00 ~ 0x3AFF
    2. BUZZER STATE : 0x3B00 ~ 0x3BFF
    3. PWM STATE    : 0x3C00 ~ 0x3CFF

1. CONSOL TASK  : 0xF000 ~
    2. TIMER        : 0xF000 ~ 0xF0FF
    3. EVENT        : 0xF100 ~ 0xF1FF
    4. COMMAND      : 0xF200 ~ 0xF2FF
    5. RESPONSE     : 0xF300 ~ 0xF3FF
===================================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Application includes. */
#include "comdef.h"

/* TASK SIGNAL INCLUDE */
#include "mctask_signals.h"
#include "consoltask_signals.h"
#include "OutEventtask_signals.h"
#include "InEventtask_signals.h"

/*===============================================================================================*/
#endif  /* SIGNALS_H */
