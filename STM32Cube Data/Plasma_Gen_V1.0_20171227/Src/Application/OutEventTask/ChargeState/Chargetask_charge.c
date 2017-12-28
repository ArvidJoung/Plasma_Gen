/*===============================================================================================*/
/**
 *   @file chargetask_charge.c
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
    
// charge state
typedef enum
{
    CHARGE_ENTRY = CHARGESTATE_CHARGE,
    CHARGE_HANDLE,
    NO_STATE
} chargetask_charge_state_type;
    

/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/


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
uint16_t charge_charge_state ( command_type *cmdptr )
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
            case CHARGE_ENTRY :
                STATUS_LED_ON(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_OFF(STA_LED_B);
                charge_state = CHARGE_HANDLE;
                break;

            case CHARGE_HANDLE :
                switch ( cmdptr->cmd )
                {
                    case CHARGE_EXTPWR_OUT_EVT:
                    case CHARGE_CHARGING_STOPED_EVT:
                        ret_state = CHARGESTATE_IDLE;
                        break;

                    case CHARGE_FULL_CHARGE_EVT:
                        ret_state = CHARGESTATE_FULL;
                        break;
                        
                    default:
                        break;
                }
                cmdptr->cmd = 0;
                break;

            default :
                DBGERR ( OUTEVENT, "bad charge state = 0x%x, cmd = 0x%x\n", charge_state, cmdptr->cmd );
                cmdptr->cmd = 0;
                break;
        }
    }

    return ret_state;
}


/*===============================================================================================*/