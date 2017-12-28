#ifndef MCTASK_SIGNALS_H
#define MCTASK_SIGNALS_H
/*===============================================================================================
 *
 *   @file mctask_signals.h
 *
 *   @version v1.0
 *
=================================================================================================

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

    1. MC TASK      : 0x1000 ~
    2. TIMER        : 0x1000 ~ 0x10FF
    3. EVENT        : 0x1100 ~ 0x11FF
    4. COMMAND      : 0x1200 ~ 0x12FF
    5. RESPONSE     : 0x1300 ~ 0x13FF

*****************************************************/

/* MC COMMANDS. */
typedef enum
{
    MC_DUMMY_TIMER  = 0x1000,
	MC_TIMER_GEN_F,
	MC_PWR_KEY_CHECK_TIMER_F,

    MC_DUMMY_EVENT  = 0x1100,
    PWR_KEY_PRESSED_EVT,
    MC_EXTPWR_IN_EVT,
    MC_EXTPWR_OUT_EVT,
    MC_LOWBATT_OCCURR_EVT,
    MC_CHARGING_STARTED_EVT,
    MC_FULL_CHARGE_EVT,
    MC_CHARGING_STOPED_EVT,

    MC_TASK_F       = 0x1200,
    MC_RESET_CMD_F,
    MC_POWEROFF_CMD_F,
    MC_STOP_CHARGE_F,
    MC_TASK_STOP_F,

    MC_OPERATE_ON_F,
    MC_OPERATE_OFF_F,
    MC_OPERATE_VOL_F,


    MC_MAX_COMMAND = 0x1FFF
} mctask_signal_type;

/*===============================================================================================*/
#endif  /* MCTASK_SIGNALS_H*/
