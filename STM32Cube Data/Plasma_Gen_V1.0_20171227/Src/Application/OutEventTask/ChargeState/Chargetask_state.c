/*===============================================================================================*/
/**
 *   @file charge_state.c
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
*/

/*===============================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Standard includes. */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Application includes. */
#include "target.h"
#include "signals.h"
#include "command.h"
#include "task_cfg.h"
#include "Chargetask_state.h"
#include "debugmsgcli.h"
#include "interface.h"
#include "task_cmds.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/
uint16_t    charge_state = CHARGESTATE_INIT;
TimerHandle_t xledBlinkTimer = NULL;

/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

void Chargetask_init(TimerCallbackFunction_t pxCallbackFunction)
{
	/* Create a one-shot timer for use later on in this test. */
	xledBlinkTimer = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								250,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )CHARGE_TIMER_Blink_F,	/* The timer identifier. */
								pxCallbackFunction );	/* The callback to be called when the timer expires. */

    // Start Charge module trigger
    sendtoOutEventCmd(CHARGE_EVENT_START_F);
}

void status_led_ctr(boolean onoff, uint8_t chan)
{
    if(onoff == ON)
    {
        switch(chan)
        {
            case 0: STATUS_LED_ON(STA_LED_R); break;
            case 1: STATUS_LED_ON(STA_LED_G); break;
            case 2: STATUS_LED_ON(STA_LED_B); break;
            default: break;
        }
    }
    else
    {
        switch(chan)
        {
            case 0: STATUS_LED_OFF(STA_LED_R); break;
            case 1: STATUS_LED_OFF(STA_LED_G); break;
            case 2: STATUS_LED_OFF(STA_LED_B); break;
            default: break;
        }
    }
}
    
boolean CheckIsChargeCmd(command_type *command)
{
    boolean         isprocessed = FALSE;
    uint8_t         *data = (uint8_t *)command->msg;
    
    if(ISCHARGESTATE_EVENT(command->cmd))
    {
        isprocessed = TRUE;
        
        switch(command->cmd)
        {
            case CHARGE_TASK_STOP_F:
                charge_state = CHARGESTATE_TEST;
                charge_state_machine(command);
                break;

            case CHARGE_STATUS_ON_F:
                status_led_ctr(ON, (uint8_t)data[FDPOS(charge_cmd_msg_field_type,Cmd)]);
                break;

            case CHARGE_STATUS_OFF_F:
                status_led_ctr(OFF, (uint8_t)data[FDPOS(charge_cmd_msg_field_type,Cmd)]);
                break;

            case CHARGE_STATUS_ALL_OFF_F:
                STATUS_LED_OFF(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_OFF(STA_LED_B);
                break;
                
			default:
                charge_state_machine(command);
				break;
        }
    }

    return isprocessed;
}    

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/
void charge_state_machine (command_type *cmdptr)
{
    uint16_t  new_state;

    do
    {
        new_state = CHARGESTATE_NOSTATE;

        switch ( CHARGESTATE_STATE( charge_state) )
        {
            case CHARGESTATE_INIT :
                new_state = charge_init_state ( cmdptr );        /* Process the initialization state */
                break;

            case CHARGESTATE_IDLE :
                new_state = charge_idle_state ( cmdptr );        /* Process the idle state */
                break;

            case CHARGESTATE_CHARGE :
                new_state = charge_charge_state ( cmdptr );        /* Process the charge state */
                break;

            case CHARGESTATE_LOWBATT :
                new_state = charge_lowbatt_state ( cmdptr );        /* Process the lowbatt state */
                break;

            case CHARGESTATE_FULL :
                new_state = charge_full_state ( cmdptr );        /* Process the full charge state */
                break;

            case CHARGESTATE_TEST :
                new_state = charge_test_state ( cmdptr );        /* Process the test state */
                break;

            default:
                DBGERR(OUTEVENT, "bad state = 0x%x, cmd = 0x%x\n", charge_state, cmdptr->cmd );
                new_state = CHARGESTATE_NOSTATE;
                break;
        }

        if ( new_state != CHARGESTATE_NOSTATE )
        {
            charge_state = new_state;
        }

    } while ( new_state != CHARGESTATE_NOSTATE );
}


/*===============================================================================================*/
