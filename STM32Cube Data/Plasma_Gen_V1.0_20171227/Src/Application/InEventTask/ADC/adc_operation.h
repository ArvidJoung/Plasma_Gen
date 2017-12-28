#ifndef ADC_OPERATION_H_
#define ADC_OPERATION_H_
/*===============================================================================================*/
/**
 *   @file adc_operation.h
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

/* Application include files. */
#include "ringbuf.h"

/*=================================================================================================
 CONSTANTS
=================================================================================================*/
#define ADC_PLA_VOL             2
#define ADC_BATT                3
#define ADC_CHARGE              4
#define ADC_BATT_TEMP           5


/*
    PA1     CHAN1   
    PB1     CHAN9   Charging Status
    PC0     CHAN10  
    PC1     CHAN11  Battery
    PC2     CHAN12  Battery Temp
    PC3     CHAN13  PLA Control
    PC5     CHAN15  
*/

/* PC1     CHAN11  Battery */
extern uint8_t     BatteryStatus;

/* PB0     CHAN9   Charging Status*/
extern uint8_t     ChargingStatus;

/* PC2     CHAN12  Battery Temp*/
extern uint8_t     BatteryTemp;

/* PC3     CHAN13  PLA Control*/
extern uint8_t     Plasma_vol;

/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
boolean adcStartAdc(uint8_t chan);
boolean Read_ADC_value(uint8_t chan);

/*===============================================================================================*/
#endif  /* ADC_OPERATION_H_ */
