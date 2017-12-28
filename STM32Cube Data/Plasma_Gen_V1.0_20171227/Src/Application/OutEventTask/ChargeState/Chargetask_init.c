/*===============================================================================================*/
/**
 *   @file chargetask_init.c
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
    INIT_ENTRY = CHARGESTATE_INIT,
    INIT_DEVICE_INIT,
    INIT_HANDLE,
    INIT_EXIT,
    NO_STATE
} chargetask_init_state_type;
    

/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/
uint8_t led_cycle=0;


/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/

/*-----------------------------------------------------------*/
boolean ledOnCycling(void)
{
    boolean ret_status = FALSE;
    
    switch(led_cycle++)
    {
        case 0:
            STATUS_LED_ON(STA_LED_R);
            STATUS_LED_OFF(STA_LED_G);
            STATUS_LED_OFF(STA_LED_B);
            break;

        case 1:
            STATUS_LED_OFF(STA_LED_R);
            STATUS_LED_ON(STA_LED_G);
            STATUS_LED_OFF(STA_LED_B);
            break;

        case 2:
            STATUS_LED_OFF(STA_LED_R);
            STATUS_LED_OFF(STA_LED_G);
            STATUS_LED_ON(STA_LED_B);
            
 // for line test
#if !defined (FOR_LINE_TEST)
            //HAL_Delay(200); 
            vTaskDelay(200);
            ret_status = TRUE;
#endif
            break;
            
    }

    led_cycle %= 3;

    return ret_status;
}



/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/
uint16_t charge_init_state ( command_type *cmdptr )
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
            case INIT_ENTRY :
                xTimerStart( xledBlinkTimer, 0 );
                charge_state = INIT_HANDLE;
                break;

            case INIT_HANDLE :
                
                switch ( cmdptr->cmd )
                {
                    case CHARGE_TIMER_Blink_F:
                        if(ledOnCycling() == FALSE)
                        {
                            xTimerStart( xledBlinkTimer, 0 );
                        }
                        else
                        {
                            new_state = INIT_EXIT;
                        }
                        break;

                    default:
                        break;
                }
                break;

            case INIT_EXIT:
                
                STATUS_LED_OFF(STA_LED_R);
                STATUS_LED_OFF(STA_LED_G);
                STATUS_LED_ON(STA_LED_B);
                
                ret_state = CHARGESTATE_IDLE;
                break;

                
            default :
                DBGERR ( OUTEVENT, "bad init state = 0x%x, cmd = 0x%x\n", charge_state, cmdptr->cmd );
                cmdptr->cmd = 0;
                break;
        }
    }

    return ret_state;
}


/*===============================================================================================*/
