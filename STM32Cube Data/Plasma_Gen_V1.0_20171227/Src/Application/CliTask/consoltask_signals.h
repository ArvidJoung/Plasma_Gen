#ifndef CONSOLTASK_SIGNALS_H
#define CONSOLTASK_SIGNALS_H
/*===============================================================================================
 *
 *   @file consoltask_signals.h
 *
 *   @version v1.0
 *
 *=================================================================================================

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------

Portability:
Indicate if this module is portable to other compilers or
platforms. If not, indicate specific reasons why is it not portable.
*/

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/* Enumerations for substates                                              */

/*****************************************************
                   SIGNAL GROUP

    1. CONSOL TASK  : 0xF000 ~
    2. TIMER        : 0xF000 ~ 0xF0FF
    3. EVENT        : 0xF100 ~ 0xF1FF
    4. COMMAND      : 0xF200 ~ 0xF2FF
    5. RESPONSE     : 0xF300 ~ 0xF3FF

*****************************************************/

/* CONSOL COMMANDS. */
typedef enum
{
    CONSOL_DUMMY_TIMER  = 0xF000,
    CONSOL_TIMER_1SEC_F,

    CONSOL_DUMMY_EVENT  = 0xF100,
    CONSOL_RX_MSG_EVT,
    CONSOL_TX_COMPLETE_EVT,
    CONSOL_RX_COMPLETE_EVT,
    CONSOL_RXING_COMPLETE_EVT,

    CONSOL_TASK_F       = 0xF200,
    CONSOL_TASK_START_CMD_F,
    CONSOL_RESET_CMD_F,
    CONSOL_TASK_STOP_F,
    CONSOL_SEND_MSG_F,

    CONSOL_MAX_COMMAND = 0xFFFF
} consoltask_signal_type;

/*===============================================================================================*/
#endif  /* CONSOLTASK_SIGNALS_H */
