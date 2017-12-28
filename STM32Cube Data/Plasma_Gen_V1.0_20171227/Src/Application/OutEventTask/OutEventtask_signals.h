#ifndef OUTEVENTTASK_SIGNALS_H
#define OUTEVENTTASK_SIGNALS_H
/*===============================================================================================
 *
 *   @file OutEventtask_signals.h
 *
 *   @version v1.0
 *
 *===============================================================================================

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------

Portability:
Indicate if this module is portable to other compilers or
platforms. If not, indicate specific reasons why is it not portable.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/* Enumerations for substates                                              */

/*****************************************************
                   SIGNAL GROUP

    1. OUTEVENT TASK  : 0x3000 ~
    2. TIMER        : 0x3000 ~ 0x30FF
    3. EVENT        : 0x3100 ~ 0x31FF
    4. COMMAND      : 0x3200 ~ 0x32FF
    5. RESPONSE     : 0x3300 ~ 0x33FF

    1. CHARGE STATE : 0x3A00 ~ 0x3AFF
    2. BUZZER STATE : 0x3B00 ~ 0x3BFF
    3. PWM STATE    : 0x3C00 ~ 0x3CFF
*****************************************************/

/* LED COMMANDS. */
typedef enum
{
    OUTEVENT_DUMMY_TIMER  = 0x3000,
    OUTEVENT_TIMER_1SEC_F,

    OUTEVENT_DUMMY_EVENT  = 0x3100,


    OUTEVENT_TASK_F       = 0x3200,
    OUTEVENT_TASK_START_CMD_F,
    OUTEVENT_TASK_STOP_F,


	// Start Charge Event
	CHARGE_EVENT_START_F  = 0x3A00,
    CHARGE_TIMER_1SEC_F,
    CHARGE_TIMER_Blink_F,
    CHARGE_RESET_CMD_F,
    CHARGE_TASK_STOP_F,

    CHARGE_TEST_STOP_F,

    CHARGE_ONOFF_F,
    CHARGE_ON_F,
    CHARGE_OFF_F,
    CHARGE_BRIGHT_F,
    CHARGE_FREQ_F,

    CHARGE_STATUS_ON_F,
    CHARGE_STATUS_OFF_F,
    CHARGE_STATUS_ALL_OFF_F,
    CHARGE_NORMAL_MODE_F,

    CHARGE_LOWBATT_OCCURR_EVT,
    CHARGE_CHARGING_STARTED_EVT,
    CHARGE_FULL_CHARGE_EVT,
    CHARGE_CHARGING_STOPED_EVT,
    CHARGE_EXTPWR_IN_EVT,
    CHARGE_EXTPWR_OUT_EVT,

	CHARGE_EVENT_MAX_COMMAND  = 0x3AFF,

	// Start Buzzer Event
	BUZZER_EVENT_START_F  = 0x3B00,
	BUZZER_TIMER_1_F,
	BUZZER_ON_F,
	BUZZER_OFF_F,
	BUZZER_SOUND_ON_F,
	BUZZER_SOUND_OFF_F,
	BUZZER_EVENT_MAX_COMMAND  = 0x3BFF,

	// Start PLASMA Event
	PLASMA_EVENT_START_F  = 0x3C00,    
	PLASMA_ONOFF_TIMER_F,
    PLASMA_LED_ON_F,
    PLASMA_LED_OFF_F,
	PLASMA_VOL_CTRL_F,
	PLASMA_START_CMD_F,
	PLASMA_STOP_CMD_F,
	PLASMA_EVENT_MAX_COMMAND  = 0x3CFF,

    OUTEVENT_MAX_COMMAND = 0x3FFF
} outeventtask_signal_type;

#define ISCHARGESTATE_EVENT(cmd)    ((cmd & 0xFF00) == CHARGE_EVENT_START_F )
#define ISBUZZERSTATE_EVENT(cmd)    ((cmd & 0xFF00) == BUZZER_EVENT_START_F )
#define ISPLASMASTATE_EVENT(cmd)    ((cmd & 0xFF00) == PLASMA_EVENT_START_F )


/*===============================================================================================*/
#endif  /* OUTEVENTTASK_SIGNALS_H */
