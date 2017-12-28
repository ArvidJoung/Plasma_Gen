/*===============================================================================================*/
/**
 *   @file InEventtask.c
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
//#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

/* Application include files. */
#include "target.h"
#include "command.h"
#include "signals.h"
#include "consol.h"
#include "task_cfg.h"
#include "timers.h"
#include "debugmsgcli.h"
#include "comdef.h"
#include "interface.h"
#include "comdef.h"
#include "InEventcli.h"
#include "ADC\adc_operation.h"
#include "InEventProcess.h"
#include "task_cmds.h"
#include "timergen.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
void vINEVENTTask( void *pvParameters );
    
/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/
#define InEventTIMER_QUEUE_LENGTH 10
#define xADC_QUEUE_LENGTH         10


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/

static QueueHandle_t xInEventTimerQueue;

/*-----------------------------------------------------------*/
/* The handle of the queue set to which the queues are added. */
static QueueSetHandle_t xQueueSet;
static QueueHandle_t xAdcQueue;

//static TimerHandle_t xineventTimer1 = NULL;
static TimerHandle_t xsensorCheckPeriodicTimer = NULL;
static TimerHandle_t xvolCheckPeriodicTimer = NULL;
static TimerHandle_t xkeyCheckTimer = NULL;
#if defined (ADC_USE_POLLING_METHOD)
    static TimerHandle_t xadcConvCheckTimer = NULL;
#endif

/*-----------------------------------------------------------*/

/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/

/*-----------------------------------------------------------*/
void prvInEventTimerCallback( TimerHandle_t pxExpiredTimer )
{
	uint32_t ulTimerID;

    portENTER_CRITICAL();
	ulTimerID = ( uint32_t ) pvTimerGetTimerID( pxExpiredTimer );
	xQueueSend(xInEventTimerQueue,&ulTimerID,0);
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

        if( xActivatedMember == xInEventTaskQueue )
        {
            xQueueReceive( xActivatedMember, &cmd, 0 );
        }
        else if( xActivatedMember == xInEventTimerQueue )
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
        
		if(cmd.cmd == INEVENT_TASK_START_CMD_F)
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
void ineventtask_init(QueueSetMemberHandle_t  xQueue)
{
	InEventCLIregister();
	waitTaskStartSignal(xQueue);
    xTimerStart( xsensorCheckPeriodicTimer, 0 );    
    xTimerStart( xvolCheckPeriodicTimer, 0 );
    InEventCheckUSBDetectPin();
}

/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
void vStartInEventTasks( void )
{
	/*First Create the queue set such that it will be able to hold a message for
	every space in every queue in the set. */
	xQueueSet = xQueueCreateSet( taskINEVENT_QUEUE_LENGTH + InEventTIMER_QUEUE_LENGTH );

	xInEventTaskQueue = xQueueCreate( taskINEVENT_QUEUE_LENGTH, sizeof( command_type ) );
    configASSERT( xInEventTaskQueue );
	xInEventTimerQueue = xQueueCreate( InEventTIMER_QUEUE_LENGTH, sizeof( uint32_t ) );
    configASSERT( xInEventTimerQueue );

	xQueueAddToSet( xInEventTaskQueue, xQueueSet );
	xQueueAddToSet( xInEventTimerQueue, xQueueSet );


    // for Queued ADC Conversion
    xAdcQueue = xQueueCreate( xADC_QUEUE_LENGTH,   sizeof( uint8_t ) );


#if defined (ADC_USE_POLLING_METHOD)
    xadcConvCheckTimer = xTimerCreate( "",              /* Text name to facilitate debugging.  The kernel does not use this itself. */
                                5,                      /* The period for the timer(5 msec). */
                                pdFALSE,                /* Don't auto-reload - hence a one shot timer. */
                                ( void * )INEVENT_ADC_CONV_CHECK_TIMER_F,  /* The timer identifier. */
                                prvInEventTimerCallback );   /* The callback to be called when the timer expires. */
#endif

	/* Create a one-shot timer for use later on in this test. */
	xkeyCheckTimer = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								50,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )INEVENT_KEY_CHECK_TIMER_F,	/* The timer identifier. */
								prvInEventTimerCallback );	/* The callback to be called when the timer expires. */

    xsensorCheckPeriodicTimer = xTimerCreate( "",       /* Text name to facilitate debugging.  The kernel does not use this itself. */
                                1000,                   /* The period for the timer(500 msec). */
                                pdFALSE,                /* Don't auto-reload - hence a one shot timer. */
                                ( void * )INEVENT_PERIODIC_TIMER_F,  /* The timer identifier. */
								prvInEventTimerCallback );   /* The callback to be called when the timer expires. */
    
    xvolCheckPeriodicTimer = xTimerCreate( "",       /* Text name to facilitate debugging.  The kernel does not use this itself. */
                            150,                   /* The period for the timer(500 msec). */
                            pdFALSE,                /* Don't auto-reload - hence a one shot timer. */
                            ( void * )INEVENT_VOL_CHECK_TIMER_F,  /* The timer identifier. */
							prvInEventTimerCallback );   /* The callback to be called when the timer expires. */

	/* Spawn the task. */
	xTaskCreate( vINEVENTTask, "LED", taskINEVENT_TASK_STACK_SIZE, xQueueSet, taskINEVENT_TASK_PRIORITY, ( TaskHandle_t * ) NULL );

}


/*-----------------------------------------------------------*/
void vINEVENTTask( void *pvParameters )
{
	//QueueHandle_t   xQueue;		// mctask가 queue 하나만 사용 할때
	QueueSetMemberHandle_t  xQueue;
	QueueSetMemberHandle_t	xActivatedMember;
	command_type	        cmd;
	uint8_t			        *data = NULL;
    uint32_t                timerid;
    uint8_t         		adc_cmd, current_adc;
    boolean         		isAdcStarted = FALSE;

	xQueue = ( QueueHandle_t * ) pvParameters;
    configASSERT( xQueue );

	ineventtask_init(xQueue);
	
	for(;;)
	{
		xActivatedMember =  xQueueSelectFromSet( xQueue, queueSHORT_DELAY );

        if( xActivatedMember == xInEventTaskQueue )
        {
            xQueueReceive( xActivatedMember, &cmd, 0 );
            data = (uint8_t *)cmd.msg;
        }
        else if( xActivatedMember == xInEventTimerQueue )
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

            case INEVENT_KEY_CHECK_TIMER_F:
                if( InEventCheckDebounce() > 0)
                {
                    xTimerStart(xkeyCheckTimer,0);   
                }
                break;

            case INEVENT_VOL_CHECK_TIMER_F:
                xTimerStart( xvolCheckPeriodicTimer, 0 );
                
                sendtoInEventCmd(INEVENT_READ_ETC_ADC_F);
                break;

            case INEVENT_READ_ETC_ADC_F:
                portENTER_CRITICAL();
                adc_cmd = ADC_PLA_VOL;
                xQueueSend(xAdcQueue, &adc_cmd, 0);
                portEXIT_CRITICAL();
                sendtoInEventCmd(INEVENT_START_ADC_F);
                break;
                
            case INEVENT_PERIODIC_TIMER_F:
                xTimerStart( xsensorCheckPeriodicTimer, 0 );

                sendtoInEventCmd(INEVENT_READ_BATT_ADC_F);

                break;

            case INEVENT_READ_BATT_ADC_F:
                portENTER_CRITICAL();
                adc_cmd = ADC_BATT;
                xQueueSend(xAdcQueue, &adc_cmd, 0);
                portEXIT_CRITICAL();

                portENTER_CRITICAL();
                adc_cmd = ADC_CHARGE;
                xQueueSend(xAdcQueue, &adc_cmd, 0);
                portEXIT_CRITICAL();

                portENTER_CRITICAL();
                adc_cmd = ADC_BATT_TEMP;
                xQueueSend(xAdcQueue, &adc_cmd, 0);
                portEXIT_CRITICAL();

                sendtoInEventCmd(INEVENT_START_ADC_F);
                break;

            case INEVENT_START_ADC_F:
                // check queue // 있으면 시작
                if(isAdcStarted == FALSE)
                {
                    if( xQueueReceive( xAdcQueue, &current_adc, 0 ) == pdPASS )
                    {
                        if( adcStartAdc(current_adc) == TRUE)
                        {
                            isAdcStarted = TRUE;
                            #if defined (ADC_USE_POLLING_METHOD)
                                xTimerStart( xadcConvCheckTimer, 0 );
                            #endif
                        }
                        else
                        {
                        	sendtoInEventCmd(INEVENT_START_ADC_F);  // next queue
                        }
                    }
                }
                break;

            case ADC_CONV_OUTOFWIN_F:
            case ADC_ERROR_OCCUR_F:                
                DBGERR(INEVENT,"ADC Error !!! (%x).\r\n",cmd.cmd);
                break;

#if defined (ADC_USE_POLLING_METHOD)
            case INEVENT_ADC_CONV_CHECK_TIMER_F:
#endif

#if defined (ADC_USE_IT_METHOD)
            case ADC_CONV_COMPLETE_F:
#endif
                Read_ADC_value(current_adc);
                isAdcStarted = FALSE;

                if(current_adc == ADC_PLA_VOL)
                {
                    InEventPlasmaVolResultReport();
                }
                else
                {
                    InEventBattResultReport(current_adc);
                }
                sendtoInEventCmd(INEVENT_START_ADC_F);  // next queue
                break;

            case INEVENT_OPERATE_SW_EVT:
                // sendMcCommand  MC_OPERATE_ON_F or MC_OPERATE_OFF_F
                // INEVENT_KEY_CHECK_TIMER_F 참조
                // glitching checking required
                InEventCheckOPSWStart();
                xTimerStop(xkeyCheckTimer,0);              
                xTimerStart(xkeyCheckTimer,0);   
                DBGLOW(INEVENT,"OP Switch Pressed.\r\n");
                break;

            case INEVENT_EXTPWR_IN_EVT:
                IsExtPwrInState = TRUE;
                sendMcCommand(MC_EXTPWR_IN_EVT);
                DBGHI(INEVENT,"External Power In.\r\n");
                break;

            case INEVENT_EXTPWR_OUT_EVT:
                IsExtPwrInState = FALSE;
                sendMcCommand(MC_EXTPWR_OUT_EVT);
                DBGHI(INEVENT,"External Power Out.\r\n");
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

