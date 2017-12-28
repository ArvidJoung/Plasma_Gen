/*===============================================================================================*/
/**
 *   @file chargetask_idle.c
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
    
// init state
typedef enum
{
    IDLE_ENTRY = CHARGESTATE_IDLE,
    IDLE_HANDLE,

    NO_STATE
} chargetask_idle_state_type;
    

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
uint16_t charge_idle_state ( command_type *cmdptr )
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
            case IDLE_ENTRY :
                STATUS_LED_OFF(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_ON(STA_LED_B);

                // TODO
                // Charge FULL 인지 Normal 인지 check필요 
                charge_state = IDLE_HANDLE;
                break;

            case IDLE_HANDLE :
                switch ( cmdptr->cmd )
                {
                    //case MC_EXTPWR_IN_EVT:
                    case CHARGE_CHARGING_STARTED_EVT:
                        ret_state = CHARGESTATE_CHARGE;
                        break;

                    case CHARGE_LOWBATT_OCCURR_EVT:
                        ret_state = CHARGESTATE_LOWBATT;
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
                DBGERR ( OUTEVENT, "bad idle state = 0x%x, cmd = 0x%x\n", charge_state, cmdptr->cmd );
                cmdptr->cmd = 0;
                break;
        }
    }

    return ret_state;
}


/*===============================================================================================*/
