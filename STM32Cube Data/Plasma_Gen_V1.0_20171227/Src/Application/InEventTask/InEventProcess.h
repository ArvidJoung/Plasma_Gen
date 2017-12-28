#ifndef INEVENT_PROCESS_H_
#define INEVENT_PROCESS_H_
/*===============================================================================================*/
/**
 *   @file InEventProcess.h
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
#include "target.h"
#include "command.h"

/* Scheduler include files. */

/* Application include files. */
#include "interface.h"
#include "InEventProcess.h"

/*=================================================================================================
 CONSTANTS
=================================================================================================*/
#define BATT_TEMP_CHARGE_LOW_LIMIT      83
#define BATT_TEMP_CHARGE_HIGH_LIMIT     189

#define BATT_TEMP_DISCHARGE_LOW_LIMIT   59
#define BATT_TEMP_DISCHARGE_HIGH_LIMIT  226

#define BATT_CUTOFF_VOLT_VAL            194//191
#define BATT_LOW_VOLT_VAL               200//197
#define BATT_FULL_VOLT_VAL              240//245

#define CHARGEADC_LOW   10
#define CHARGEADC_HIGH  150

/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
extern boolean IsExtPwrInState;

uint8_t InEventCheckDebounce(void);
void InEventCheckOPSWStart(void);
void InEventBattResultReport(uint8_t adc_chan);
void InEventPlasmaVolResultReport(void);
void InEventCheckUSBDetectPin(void);

/*===============================================================================================*/
#endif  /* INEVENT_PROCESS_H_ */
