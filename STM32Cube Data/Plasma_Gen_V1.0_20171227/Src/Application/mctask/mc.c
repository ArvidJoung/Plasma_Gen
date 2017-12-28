/*
        1 tab == 4 spaces!
*/

/*===============================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Standard includes. */
#include <stdlib.h>


/* Scheduler include files. */
#include "FreeRTOS.h"
//#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

/* Application include files. */
#include "command.h"
#include "signals.h"
#include "consol.h"
#include "task_cfg.h"
#include "timers.h"
#include "timergen.h"
#include "debugmsgcli.h"
#include "mccli.h"
#include "task_cmds.h"
#include "mc_process.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/* The task that is created eight times - each time with a different xLEDParaemtes 
structure passed in as the parameter. */
static void vMCTask( void *pvParameters );


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/

#define mcTIMER_QUEUE_LENGTH 20


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/

static QueueHandle_t xmcTimerQueue;

static uint16_t pwrKeyCount=0;


/*-----------------------------------------------------------*/
/* The handle of the queue set to which the queues are added. */
static QueueSetHandle_t xQueueSet;

TimerHandle_t xmcTimergen = NULL;
TimerHandle_t xmcPwrKeyCheckTimer = NULL;

/*-----------------------------------------------------------*/

/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/

static void prvmcTimerCallback( TimerHandle_t pxExpiredTimer )
{
    uint32_t ulTimerID;

    portENTER_CRITICAL();
	ulTimerID = ( uint32_t ) pvTimerGetTimerID( pxExpiredTimer );
	xQueueSend(xmcTimerQueue,&ulTimerID,0);
    portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/
void mctask_init()
{
	mcCLIregister();

	check_pwronstate();

	sendTaskStartSignal();
}


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
void vStartMCTasks( void )
{
	/*First Create the queue set such that it will be able to hold a message for
	every space in every queue in the set. */
	xQueueSet = xQueueCreateSet( taskMC_QUEUE_LENGTH + mcTIMER_QUEUE_LENGTH );

	xMcTaskQueue = xQueueCreate( taskMC_QUEUE_LENGTH, sizeof( command_type ) );
    configASSERT( xMcTaskQueue );
	xmcTimerQueue = xQueueCreate( mcTIMER_QUEUE_LENGTH, sizeof( uint32_t ) );
    configASSERT( xmcTimerQueue );

	xQueueAddToSet( xMcTaskQueue, xQueueSet );
	xQueueAddToSet( xmcTimerQueue, xQueueSet );

	xmcTimergen = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								1000,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )MC_TIMER_GEN_F,	/* The timer identifier. */
								prvmcTimerCallback );	/* The callback to be called when the timer expires. */

	/* Create a one-shot timer for use later on in this test. */
	xmcPwrKeyCheckTimer = xTimerCreate(	"",		/* Text name to facilitate debugging.  The kernel does not use this itself. */
								500,					/* The period for the timer(500 msec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )MC_PWR_KEY_CHECK_TIMER_F,	/* The timer identifier. */
								prvmcTimerCallback );	/* The callback to be called when the timer expires. */

	/* Spawn the task. */
	xTaskCreate( vMCTask, "MCx", taskMC_TASK_STACK_SIZE, xQueueSet, taskMC_TASK_PRIORITY, ( TaskHandle_t * ) NULL );

}


/*-----------------------------------------------------------*/


static void vMCTask( void *pvParameters )
{
	//QueueHandle_t   xQueue;		// mctask가 queue 하나만 사용 할때
	QueueSetMemberHandle_t  xQueue;
	QueueSetMemberHandle_t	xActivatedMember;
	command_type	        cmd;
    uint32_t                timerid;


	xQueue = ( QueueHandle_t * ) pvParameters;
    configASSERT( xQueue );

	mctask_init();
    
	xTimerStart(xmcTimergen, 0);
    
	for(;;)
	{
		xActivatedMember =  xQueueSelectFromSet( xQueue, queueSHORT_DELAY );
        if( xActivatedMember == xMcTaskQueue )
        {
            xQueueReceive( xActivatedMember, &cmd, 0 );
        }
        else if( xActivatedMember == xmcTimerQueue )
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
                DBGMED(MC,"Current MC Status is 0x%x\r\n",mctask_state);
				break;

			case MC_POWEROFF_CMD_F:
				DBGHI(MC,"MC_POWEROFF_CMD_F requested!!!\r\n");
                sendSTOPandPowerOFF();
				break;

			case MC_PWR_KEY_CHECK_TIMER_F:
				DBGLOW(MC,"MC_PWR_KEY_CHECK_TIMER_F is expired\r\n");

				if(check_pwroffstate() == TRUE)
				{
					sendMcCommand(MC_POWEROFF_CMD_F);
				}
				break;

			case PWR_KEY_PRESSED_EVT:
				DBGHI(MC,"PWR_KEY_PRESSED_EVT\r\n");
				pwrKeyCount = 0;
				xTimerStart( xmcPwrKeyCheckTimer, 0 );
				break;

            case MC_LOWBATT_OCCURR_EVT:
                sendtoBuzzerOnCmd(1);
                sendtoOutEventCmd(CHARGE_LOWBATT_OCCURR_EVT);

                // TODO 
                // 정기적으로 lowbatt check후 소리 필요 
                break;
                
            case MC_CHARGING_STARTED_EVT:
                 sendtoOutEventCmd(CHARGE_CHARGING_STARTED_EVT);
                 break;
                 
            case MC_FULL_CHARGE_EVT:
                sendtoOutEventCmd(CHARGE_FULL_CHARGE_EVT);
                break;
                
            case MC_CHARGING_STOPED_EVT:
                sendtoOutEventCmd(CHARGE_CHARGING_STOPED_EVT);
                break;
    
            case MC_EXTPWR_IN_EVT:
                sendtoBuzzerOnCmd(2);
                sendtoOutEventCmd(CHARGE_EXTPWR_IN_EVT);
                break;
                
            case MC_EXTPWR_OUT_EVT:
                sendtoBuzzerOnCmd(2);
                sendtoOutEventCmd(CHARGE_EXTPWR_OUT_EVT);
                break;

            case MC_OPERATE_ON_F:
                sendtoBuzzerOnCmd(3);
				sendtoOutEventCmd(PLASMA_LED_ON_F);
                sendtoOutEventCmd(PLASMA_START_CMD_F);
                break;
                
            case MC_OPERATE_OFF_F:
				sendtoOutEventCmd(PLASMA_LED_OFF_F);
                sendtoOutEventCmd(PLASMA_STOP_CMD_F);
                break;

            case MC_OPERATE_VOL_F:
                PlasmaVolSend2Task(&cmd);
                break;

            default:
                break;

		}

		if( (cmd.msg != NULL) && (cmd.isconst == FALSE))
		{
			cmd_mfree(cmd.msg);
		}
	}
}



/*-----------------------------------------------------------*/

