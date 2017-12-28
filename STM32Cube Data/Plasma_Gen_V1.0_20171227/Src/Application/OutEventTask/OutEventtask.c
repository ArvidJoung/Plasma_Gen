/*===============================================================================================*/
/**
 *   @file OutEventtask.c
 *
 *   @version v1.0
 */
/*================================================================================================*/


/*===============================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Standard includes. */
#include <stdlib.h>


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

/* Application include files. */
#include "command.h"
#include "signals.h"
#include "consol.h"
#include "task_cfg.h"
#include "timers.h"
#include "debugmsgcli.h"
#include "comdef.h"
#include "interface.h"
#include "tim.h"
#include "comdef.h"
#include "OutEventcli.h"
#include "task_cmds.h"
#include "Chargetask_state.h"
#include "Buzzer_state.h"
#include "Plasma_state.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
void vOUTEVENTTask( void *pvParameters );
    
/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/
#define OutEventTIMER_QUEUE_LENGTH 15

/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/

static QueueHandle_t xOutEventTimerQueue;

/*-----------------------------------------------------------*/
/* The handle of the queue set to which the queues are added. */
static QueueSetHandle_t xQueueSet;

TimerHandle_t xouteventTimer1 = NULL;

/*-----------------------------------------------------------*/

/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/

/*-----------------------------------------------------------*/
void prvOutEventTimerCallback( TimerHandle_t pxExpiredTimer )
{
	uint32_t ulTimerID;

    portENTER_CRITICAL();
	ulTimerID = ( uint32_t ) pvTimerGetTimerID( pxExpiredTimer );
	xQueueSend(xOutEventTimerQueue,&ulTimerID,0);
    portEXIT_CRITICAL();
}


/*-----------------------------------------------------------*/
static void waitTaskStartSignal(QueueSetMemberHandle_t xQueue)
{
	QueueSetMemberHandle_t	xActivatedMember;
	command_type			cmd;
    uint32_t                timerid;
    
	for(;;)
	{
		xActivatedMember =  xQueueSelectFromSet( xQueue, queueNO_DELAY );

        if( xActivatedMember == xOutEventTaskQueue )
        {
            xQueueReceive( xActivatedMember, &cmd, 0 );
        }
        else if( xActivatedMember == xOutEventTimerQueue )
        {
            xQueueReceive( xActivatedMember, &timerid, 0 );
            cmd.cmd = (uint16_t)timerid;
            cmd.len = 0;
			cmd.msg = NULL;
        }
		else
		{
			cmd.cmd = 0;
			cmd.msg = NULL;
		}
        
		if(cmd.cmd == OUTEVENT_TASK_START_CMD_F)
		{
			// start normal processing
			break;
		}
        
		if( (cmd.msg != NULL) && (cmd.isconst == FALSE))
		{
			cmd_mfree(cmd.msg);
		}
        
	}
}

/*-----------------------------------------------------------*/
void outeventtask_init(QueueSetMemberHandle_t  xQueue)
{
	OutEventCLIregister();
	waitTaskStartSignal(xQueue);
	
	Chargetask_init(prvOutEventTimerCallback);
	BuzzerState_init(prvOutEventTimerCallback);
	PlasmaState_init(prvOutEventTimerCallback);
}

/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
void vStartOutEventTasks( void )
{
	/*First Create the queue set such that it will be able to hold a message for
	every space in every queue in the set. */
	xQueueSet = xQueueCreateSet( taskOUTEVENT_QUEUE_LENGTH + OutEventTIMER_QUEUE_LENGTH );

	xOutEventTaskQueue = xQueueCreate( taskOUTEVENT_QUEUE_LENGTH, sizeof( command_type ) );
    configASSERT( xOutEventTaskQueue );    
	xOutEventTimerQueue = xQueueCreate( OutEventTIMER_QUEUE_LENGTH, sizeof( uint32_t ) );
    configASSERT( xOutEventTimerQueue );

	xQueueAddToSet( xOutEventTaskQueue, xQueueSet );
	xQueueAddToSet( xOutEventTimerQueue, xQueueSet );


	/* Create a one-shot timer for use later on in this test. */
	xouteventTimer1 = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								1000,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )OUTEVENT_TIMER_1SEC_F,	/* The timer identifier. */
								prvOutEventTimerCallback );	/* The callback to be called when the timer expires. */

	/* Spawn the task. */
	xTaskCreate( vOUTEVENTTask, "LED", taskOUTEVENT_TASK_STACK_SIZE, xQueueSet, taskOUTEVENT_TASK_PRIORITY, ( TaskHandle_t * ) NULL );

}


/*-----------------------------------------------------------*/
void vOUTEVENTTask( void *pvParameters )
{
	//QueueHandle_t   xQueue;		// mctask가 queue 하나만 사용 할때
	QueueSetMemberHandle_t  xQueue;
	QueueSetMemberHandle_t	xActivatedMember;
	command_type	        cmd;
	uint8_t                 *data = NULL;
    uint32_t                timerid;
    boolean                 isprocessed = FALSE;

	xQueue = ( QueueHandle_t * ) pvParameters;
    configASSERT( xQueue );

	outeventtask_init(xQueue);
	
	for(;;)
	{
		xActivatedMember =  xQueueSelectFromSet( xQueue, queueSHORT_DELAY );

        if( xActivatedMember == xOutEventTaskQueue )
        {
            xQueueReceive( xActivatedMember, &cmd, 0 );
            data = (uint8_t *)cmd.msg;
        }
        else if( xActivatedMember == xOutEventTimerQueue )
        {
            xQueueReceive( xActivatedMember, &timerid, 0 );
            cmd.cmd = (uint16_t)timerid;
            cmd.len = 0;
			cmd.msg = NULL;
        }
		else
		{
			cmd.cmd = 0;
			cmd.msg = NULL;
		}
        
		//processing
		switch(cmd.cmd)
		{
			case 0:
				// No cmd rxed, 2000 msec wait timeout
				break;

            case OUTEVENT_TASK_STOP_F:
                sendtoOutEventCmd(BUZZER_OFF_F);
                sendtoOutEventCmd(PLASMA_STOP_CMD_F);
                sendtoOutEventCmd(PLASMA_LED_OFF_F);
                sendtoOutEventCmd(PLASMA_LED_OFF_F);  
                sendtoOutEventCmd(CHARGE_STATUS_ALL_OFF_F);
                break;
                
			default:
                if(( isprocessed = CheckIsChargeCmd(&cmd)) == TRUE)
                {
                    break;
                }
                else if(( isprocessed = CheckIsBuzzerCmd(&cmd)) == TRUE)
                {
                    break;
                }
                else if(( isprocessed = CheckIsPlasmaCmd(&cmd)) == TRUE)
                {
	                break;
                }
                else
                {
				    break;
                }
		}

		if( (cmd.msg != NULL) && (cmd.isconst == FALSE))
		{
			cmd_mfree(cmd.msg);
		}
	}
}


/*-----------------------------------------------------------*/

