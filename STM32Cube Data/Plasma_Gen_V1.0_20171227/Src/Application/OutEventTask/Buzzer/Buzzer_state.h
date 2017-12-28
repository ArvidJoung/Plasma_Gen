#ifndef _BUZZER_STATE_H_
#define _BUZZER_STATE_H_
/*===============================================================================================*/
/**
 *   @file Buzzer_state.h
 *
 *   @version v1.0
 */
/*=================================================================================================


Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------


===================================================================================================*/
/* Standard includes. */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Application include files. */
#include "command.h"


/*=================================================================================================
 CONSTANTS
=================================================================================================*/
typedef struct {
    uint32              OnePerFreq;     // usec time
    uint32              ONPulseTime;    // active during usec_time * ONPulseTime
    uint32              OFFPulseTime;   // deactive during usec_time * OFFPulseTime
} BZ_sound_data_type;

typedef struct {
    boolean                   repeat;
    const BZ_sound_data_type  *sound;
} BZ_sound_type;

/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
void BuzzerState_init(TimerCallbackFunction_t pxCallbackFunction);
boolean CheckIsBuzzerCmd(command_type *command);


/*===============================================================================================*/
#endif  /* CHARGETASK_STATE_H_ */
