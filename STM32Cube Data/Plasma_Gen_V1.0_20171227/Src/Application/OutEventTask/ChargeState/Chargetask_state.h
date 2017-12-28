#ifndef CHARGETASK_STATE_H_
#define CHARGETASK_STATE_H_
/*===============================================================================================*/
/**
 *   @file chargetask_state.h
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
/* LEDTASK main states */
#define CHARGESTATE_INIT       0x0100
#define CHARGESTATE_IDLE       0x0200
#define CHARGESTATE_CHARGE     0x0300
#define CHARGESTATE_LOWBATT    0x0400
#define CHARGESTATE_FULL       0x0500
#define CHARGESTATE_TEST       0x0600

#define CHARGESTATE_NOSTATE    0xFF00
#define CHARGESTATE_EXIT       0xFF00
    
#define CHARGESTATE_STATE(state)    (state & 0xFF00 )


#define STATUS_LED_ON(mask)	    HAL_GPIO_WritePin(mask##_GPIO_Port, mask##_Pin, GPIO_PIN_SET);
#define STATUS_LED_OFF(mask)	HAL_GPIO_WritePin(mask##_GPIO_Port, mask##_Pin, GPIO_PIN_RESET);

extern uint16_t charge_state;
extern TimerHandle_t xledBlinkTimer;


/*=================================================================================================
 FUNCTION PROTOTYPES
=================================================================================================*/
void Chargetask_init(TimerCallbackFunction_t pxCallbackFunction);
void status_led_ctr(boolean onoff, uint8_t chan);
boolean CheckIsChargeCmd(command_type *command);
void charge_state_machine ( command_type *cmdptr);
uint16_t charge_init_state ( command_type *cmdptr);
uint16_t charge_idle_state ( command_type *cmdptr);
uint16_t charge_charge_state ( command_type *cmdptr);
uint16_t charge_lowbatt_state ( command_type *cmdptr);
uint16_t charge_full_state ( command_type *cmdptr);
uint16_t charge_test_state ( command_type *cmdptr );

/*===============================================================================================*/
#endif  /* CHARGETASK_STATE_H_ */
