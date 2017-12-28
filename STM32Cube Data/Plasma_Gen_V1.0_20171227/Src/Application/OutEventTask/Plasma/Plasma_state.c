/*===============================================================================================*/
/**
 *   @file Plasma_state.c
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
#include "Plasma_state.h"
#include "debugmsgcli.h"
#include "interface.h"
#include "tim.h"
#include "timergen.h"
#include "timers.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/
	
#define LED_ON(mask)	    HAL_GPIO_WritePin(mask##_GPIO_Port, mask##_Pin, GPIO_PIN_SET);
#define LED_OFF(mask)	    HAL_GPIO_WritePin(mask##_GPIO_Port, mask##_Pin, GPIO_PIN_RESET);


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/
//#define TESTMODE

#ifdef TESTMODE
	#define PLASMA_MIN_PWM_PERIOD       3000
	#define PLASMA_MAX_PWM_PERIOD       9000

#else
	#define PLASMA_MIN_ON_TIME_PWM_MSEC 100
	#define PLASMA_MAX_ON_TIME_PWM_MSEC 900

	#define TIM1_OUTPUT_CLK				(80000)	//TIM1 output clock = 80KHz
	#define PLASMA_FREQ					((TIM1_COUNTER_CLK/TIM1_OUTPUT_CLK)-1)		// TIM1 Period
#endif

/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/
static uint8_t PlasmaVol = 0;
static boolean isPlasmaOn = FALSE;
static boolean isPlasmaOnStart = FALSE;
TimerHandle_t xplasmaOnOffTimer = NULL;

/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/
#ifdef TESTMODE
uint32_t getPlasmaPeriod(void)
{
    return (PLASMA_MIN_PWM_PERIOD + ( (PLASMA_MAX_PWM_PERIOD - PLASMA_MIN_PWM_PERIOD)/256)*PlasmaVol);
}
#else
uint32_t getPlasmaTime(boolean *isOnFlag)
{
	uint32_t ret_val, ontime;

	// on time
	ontime = (PLASMA_MIN_ON_TIME_PWM_MSEC + ( (PLASMA_MAX_ON_TIME_PWM_MSEC - PLASMA_MIN_ON_TIME_PWM_MSEC)/255)*PlasmaVol);
	if(*isOnFlag)
	{
		*isOnFlag = FALSE;
		ret_val = ontime;
	}
	else
	{
		*isOnFlag = TRUE;
		ret_val = (PLASMA_MIN_ON_TIME_PWM_MSEC + PLASMA_MAX_ON_TIME_PWM_MSEC) - ontime;
		if(ret_val == 0)
		{
			*isOnFlag = FALSE;
			ret_val = ontime;
		}
	}
    return ret_val;
}
#endif


void Plasma_Start(void)
{
#ifdef TESTMODE
    uint32_t period = getPlasmaPeriod();
    
    MX_TIM1_Init(period, period/2);
#else
    MX_TIM1_Init(PLASMA_FREQ, PLASMA_FREQ/2, TRUE);
#endif

    HAL_GPIO_WritePin(EN__8V_GPIO_Port, EN__8V_Pin, GPIO_PIN_SET);
    
    /* TIM1 enable counter */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);    
}

void Plasma_Pause(void)
{
    MX_TIM1_Init(PLASMA_FREQ, PLASMA_FREQ/2, FALSE);    
}

void Plasma_Stop(void)
{
    /* TIM1 disable counter */
    //HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1); 
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); 
    HAL_GPIO_WritePin(EN__8V_GPIO_Port, EN__8V_Pin, GPIO_PIN_RESET);
}

/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

void PlasmaState_init(TimerCallbackFunction_t pxCallbackFunction)
{

	/* Create a one-shot timer for use later on in this test. */
	xplasmaOnOffTimer = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								250,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )PLASMA_ONOFF_TIMER_F,	/* The timer identifier. */
								pxCallbackFunction );	/* The callback to be called when the timer expires. */

    // Start Plasma module trigger
    //sendtoOutEventCmd(PLASMA_EVENT_START_F);
}
    
boolean CheckIsPlasmaCmd(command_type *command)
{
    boolean         isprocessed = FALSE;
    uint8_t         *data = (uint8_t *)command->msg;
    
    if(ISPLASMASTATE_EVENT(command->cmd))
    {
        isprocessed = TRUE;
        
        switch(command->cmd)
        {
			case PLASMA_ONOFF_TIMER_F:
				if(isPlasmaOn)
				{
#ifdef TESTMODE
#else
					if(isPlasmaOnStart)
					{
					    //HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
					    //HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
                        Plasma_Start();
					}
					else
					{
                        Plasma_Pause();
					}
					timer_set_timer(xplasmaOnOffTimer,getPlasmaTime(&isPlasmaOnStart));
#endif
				}
				break;
                
            case PLASMA_LED_ON_F:
                LED_ON(OP_LED);
                DBGHI(OUTEVENT,"Plasma LED On.\r\n");
                break;
                
            case PLASMA_LED_OFF_F:
                LED_OFF(OP_LED);
                DBGHI(OUTEVENT,"Plasma LED Off.\r\n");
                break;
				
        	case PLASMA_VOL_CTRL_F:
                PlasmaVol = (uint8_t)data[FDPOS(outevent_cmd_msg_field_type,Cmd)];
#ifdef TESTMODE                
                if(isPlasmaOn)
                {
                    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1); 
                    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); 
                    Plasma_Start();
                }
#endif                
				break;
				
			case PLASMA_START_CMD_F:
                Plasma_Start();
                isPlasmaOn = TRUE;
                isPlasmaOnStart = TRUE;
#ifdef TESTMODE
#else
				timer_set_timer(xplasmaOnOffTimer,getPlasmaTime(&isPlasmaOnStart));
#endif
                DBGHI(OUTEVENT,"Plasma Start.\r\n");
				break;
						
			case PLASMA_STOP_CMD_F:
                Plasma_Stop();
                isPlasmaOn = FALSE;
                DBGHI(OUTEVENT,"Plasma Stopped.\r\n");
				break;
       
			default:
				break;
        }
    }

    return isprocessed;
}    

/*===============================================================================================*/
