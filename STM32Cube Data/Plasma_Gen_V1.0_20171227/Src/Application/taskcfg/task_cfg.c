/*===============================================================================================*/
/**
 *   @file task_cfg.c
 *
 *   @version v1.0
 */
/*=================================================================================================

 (c) Copyright, All Rights Reserved

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------
jsyoon          2015/01/05

=================================================================================================*/
/* Standard includes. */

/* Scheduler include files. */
#include "FreeRTOS.h"

/* Application include files. */
#include "queue.h"
#include "task_cfg.h"

QueueHandle_t    xMcTaskQueue;
QueueHandle_t    xInEventTaskQueue;
QueueHandle_t    xOutEventTaskQueue;
QueueHandle_t    xCliTaskQueue;