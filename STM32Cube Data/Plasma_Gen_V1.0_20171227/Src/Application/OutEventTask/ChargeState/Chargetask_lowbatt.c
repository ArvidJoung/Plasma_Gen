/*===============================================================================================*/
/**
 *   @file chargetask_lowbatt.c
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
#include "OutEventtask_signals.h"
#include "command.h"
#include "task_cfg.h"
#include "Chargetask_state.h"
#include "debugmsgcli.h"


/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
    
// lowbatt state
typedef enum
{
    LOWBATT_ENTRY = CHARGESTATE_LOWBATT,
    LOWBATT_BLINK_ON_PART,
    LOWBATT_BLINK_OFF_PART,
    LOWBATT_HANDLE,

    NO_STATE
} chargetask_lowbatt_state_type;
    

/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/
uint16_t old_state;


/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/
uint16_t charge_lowbatt_state ( command_type *cmdptr )
{
    uint16_t new_state; /* new state if any */
    uint16_t ret_state; /* return state */

    ret_state = CHARGESTATE_NOSTATE;    /* don't assume a return state */
    new_state = charge_state;

    while ( new_state != NO_STATE )
    {
        charge_state = new_state;
        new_state = NO_STATE;

        switch ( charge_state )
        {
            case LOWBATT_ENTRY :
                xTimerStart( xledBlinkTimer, 0 );

            case LOWBATT_BLINK_ON_PART:
                STATUS_LED_ON(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_OFF(STA_LED_B);
                old_state = LOWBATT_BLINK_ON_PART;
                charge_state = LOWBATT_HANDLE;
                break;

            case LOWBATT_BLINK_OFF_PART:
                STATUS_LED_OFF(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_OFF(STA_LED_B);
                old_state = LOWBATT_BLINK_OFF_PART;
                charge_state = LOWBATT_HANDLE;
                break;

                
            case LOWBATT_HANDLE :
                switch ( cmdptr->cmd )
                {
                    //case MC_EXTPWR_IN_EVT:
                    case CHARGE_CHARGING_STARTED_EVT:
                        ret_state = CHARGESTATE_CHARGE;
                        break;

                    case CHARGE_TIMER_Blink_F:
                        xTimerStart( xledBlinkTimer, 0 );
                        new_state = LOWBATT_BLINK_ON_PART + (++old_state % 2);
                        break;
                        
                    default:
                        break;
                }
                cmdptr->cmd = 0;
                break;

            default :
                DBGERR ( OUTEVENT, "bad lowbatt state = 0x%x, cmd = 0x%x\n", charge_state, cmdptr->cmd );
                cmdptr->cmd = 0;
                break;
        }
    }

    return ret_state;
}


/*===============================================================================================*/
