#ifndef INEVENTTASK_SIGNALS_H
#define INEVENTTASK_SIGNALS_H
/*===============================================================================================
 *
 *   @file InEventtask_signals.h
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

    1. INEVENT TASK  : 0x2000 ~
    2. TIMER        : 0x2000 ~ 0x20FF
    3. EVENT        : 0x2100 ~ 0x21FF
    4. COMMAND      : 0x2200 ~ 0x22FF
    5. RESPONSE     : 0x2300 ~ 0x23FF

*****************************************************/

/* LED COMMANDS. */
typedef enum
{
    INEVENT_DUMMY_TIMER  = 0x2000,
    INEVENT_KEY_CHECK_TIMER_F,
	INEVENT_PERIODIC_TIMER_F,
	INEVENT_VOL_CHECK_TIMER_F,
	INEVENT_ADC_CONV_CHECK_TIMER_F,

    INEVENT_DUMMY_EVENT  = 0x2100,
	INEVENT_BATT_VAL_EVT,
	INEVENT_BATT_TEMP_EVT,
	INEVENT_CHARGE_STATUS_ADC_EVT,
    
    INEVENT_EXTPWR_IN_EVT,
    INEVENT_EXTPWR_OUT_EVT,
    INEVENT_OPERATE_SW_EVT,
    
    INEVENT_CHARGING_STOPED_EVT,
    INEVENT_FULL_CHARGE_EVT,
    INEVENT_LOW_BATTERY_EVT,
    INEVENT_CHARGING_STARTED_EVT,
    

    INEVENT_TASK_F       = 0x2200,
    INEVENT_TASK_START_CMD_F,
    INEVENT_TASK_STOP_F,
	INEVENT_READ_BATT_ADC_F,
	INEVENT_READ_ETC_ADC_F,

	INEVENT_START_ADC_F,

	ADC_CONV_COMPLETE_F,
	ADC_CONV_OUTOFWIN_F,
	ADC_ERROR_OCCUR_F,

	INEVENT_CHARGE_NORMAL_MODE_F,

    INEVENT_MAX_COMMAND = 0x2FFF
} ineventtask_signal_type;

/*===============================================================================================*/
#endif  /* INEVENTTASK_SIGNALS_H */
