#ifndef TASK_CFG_H
#define TASK_CFG_H
/*===============================================================================================*/
/**
 *   @file task_cfg.h
 *
 *   @version v1.0
 */
/*=================================================================================================

 (c) Copyright  All Rights Reserved

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------
jsyoon          2017/12/25


=================================================================================================*/
/* Standard includes. */
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
    
/* Application include files. */



/* CONSTANTS */

#define taskNO_BLOCK        (0)
#define queueSHORT_DELAY	(2000)
#define queueNO_DELAY	    (0)


/* Priorities at which the tasks are created. */

// configMAX_PRIORITIES 7
//#define configTIMER_TASK_PRIORITY		( configMAX_PRIORITIES - 1 )
#define taskMC_TASK_PRIORITY			( configMAX_PRIORITIES - 2 )
#define taskINEVENT_TASK_PRIORITY		( configMAX_PRIORITIES - 3 )
#define taskOUTEVENT_TASK_PRIORITY		( configMAX_PRIORITIES - 4 )
#define taskCLI_TASK_PRIORITY			( configMAX_PRIORITIES - 5 )
//#define tskIDLE_PRIORITY				( ( UBaseType_t ) 0U )


/* tasks stack size. */
#define taskMC_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE * 3)
#define taskINEVENT_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE * 3)
#define taskOUTEVENT_TASK_STACK_SIZE	(configMINIMAL_STACK_SIZE * 3)
#define taskCLI_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE * 1)


/* tasks queue size. */
#define taskMC_QUEUE_LENGTH				20
#define taskINEVENT_QUEUE_LENGTH		20
#define taskOUTEVENT_QUEUE_LENGTH		20
#define taskCLI_QUEUE_LENGTH			10
 
/* Task queue */
extern QueueHandle_t    xMcTaskQueue;
extern QueueHandle_t    xInEventTaskQueue;
extern QueueHandle_t    xOutEventTaskQueue;
extern QueueHandle_t    xCliTaskQueue;

/* FUNCTION PROTOTYPES */

void vStartMCTasks( void );
void vStartCONSOLTasks( void );
void vStartInEventTasks( void );
void vStartOutEventTasks( void );

#endif  /* TASK_CFG_H */
