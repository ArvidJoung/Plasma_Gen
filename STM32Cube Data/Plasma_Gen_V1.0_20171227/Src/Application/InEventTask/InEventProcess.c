/*===============================================================================================*/
/**
 *   @file InEventProcess.c
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
#include "comdef.h"
#include "signals.h"
#include "command.h"
#include "task_cfg.h"
#include "interface.h"
#include "InEventProcess.h"
#include "ADC/adc_operation.h"
#include "debugmsgcli.h"
#include "task_cmds.h"
#include "stm32f0xx_hal.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
 LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
 LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
 LOCAL VARIABLES
==================================================================================================*/
uint8_t IsCharging = 0xFF;
boolean IsExtPwrInState = FALSE;
uint8_t ProPlasmaVol=0;

// for op key
boolean isOpSWKeyChecking = FALSE;
static uint8_t OpSWKeyCount = 0;
boolean isPlasmaOperate = FALSE;

/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/
void ProcessBattADCval(uint16_t command, uint8_t adc_val );

/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
uint8_t InEventCheckDebounce(void)
{
    uint8_t isTimerRequired = 0;

    if(isOpSWKeyChecking) 
    {
        if(HAL_GPIO_ReadPin(KEY_OP_GPIO_Port,KEY_OP_Pin) == GPIO_PIN_RESET)
        {
            OpSWKeyCount++;
            
            if(OpSWKeyCount >= 3)
            {
                isOpSWKeyChecking = FALSE;
                OpSWKeyCount = 0;
                if(isPlasmaOperate == FALSE)
                {
                    isPlasmaOperate = TRUE;
                    sendMcCommand(MC_OPERATE_ON_F);
                    DBGHI(INEVENT,"Operate Switch on state !!!\r\n");
                }
                else
                {
                    isPlasmaOperate = FALSE;
                    sendMcCommand(MC_OPERATE_OFF_F);   
                    DBGHI(INEVENT,"Operate Switch off state !!!\r\n");
                }
            }
            else
            {
                isTimerRequired++;                
            }
        }
        else
        {
            OpSWKeyCount = 0;
            isOpSWKeyChecking = FALSE;
        }
    }

    return isTimerRequired;
}

void InEventCheckOPSWStart(void)
{
    OpSWKeyCount = 1;
    isOpSWKeyChecking = TRUE;
}


void InEventCheckUSBDetectPin(void)
{
    if(HAL_GPIO_ReadPin(USB_DET_GPIO_Port,USB_DET_Pin) == GPIO_PIN_SET)
    {
        IsExtPwrInState = TRUE;
    }
}

void InEventBattResultReport(uint8_t adc_chan)
{
    if(adc_chan == ADC_BATT)
    {
        ProcessBattADCval(INEVENT_BATT_VAL_EVT, BatteryStatus);
    }
    else if (adc_chan == ADC_BATT_TEMP)
    {
        ProcessBattADCval(INEVENT_BATT_TEMP_EVT, BatteryTemp);
    }
    else if (adc_chan == ADC_CHARGE)
    {
        ProcessBattADCval(INEVENT_CHARGE_STATUS_ADC_EVT, ChargingStatus);
    }
    
}

void InEventPlasmaVolResultReport(void)
{
    if(ProPlasmaVol != Plasma_vol)
    {
        command_type    cmd;
        uint8_t         *data;
        
        ProPlasmaVol = Plasma_vol;
        
        if((data = (uint8_t *)cmd_malloc(FDPOS(outevent_cmd_msg_field_type,EndofData))) != NULL)
        {
            portENTER_CRITICAL();
            cmd.len = FDPOS(outevent_cmd_msg_field_type,EndofData);
            cmd.isconst = FALSE;
            cmd.cmd = MC_OPERATE_VOL_F;
            data[FDPOS(outevent_cmd_msg_field_type,Cmd)] = ProPlasmaVol;
            cmd.msg = data;
            xQueueSend(xMcTaskQueue,&cmd,taskNO_BLOCK);
            portEXIT_CRITICAL();
        }        
    }   
}

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/
void ProcessBattADCval(uint16_t command, uint8_t adc_val )
{
    switch(command)
    {
        case INEVENT_BATT_TEMP_EVT:
            {
                uint8_t low_limit, high_limit;
                
                if(IsCharging == 1)
                {
                    low_limit = BATT_TEMP_CHARGE_LOW_LIMIT;
                    high_limit = BATT_TEMP_CHARGE_HIGH_LIMIT;
                }
                else
                {
                    low_limit = BATT_TEMP_DISCHARGE_LOW_LIMIT;
                    high_limit = BATT_TEMP_DISCHARGE_HIGH_LIMIT;
                }

                {
                    if( (adc_val < low_limit) || (adc_val > high_limit))
                    {
                        DBGERR(INEVENT,"Battery Range out of bound Error!!!\r\n");
    					sendMcCommand(MC_POWEROFF_CMD_F);
                    }
                }

            }
            break;

        case INEVENT_BATT_VAL_EVT:
            if( (IsExtPwrInState == FALSE) && (adc_val < BATT_CUTOFF_VOLT_VAL))
            {
                DBGERR(INEVENT,"Battery Cutoff Event occurred!!!\r\n");
                sendMcCommand(MC_POWEROFF_CMD_F);
            }
            else if( (IsExtPwrInState == FALSE) && (adc_val < BATT_LOW_VOLT_VAL))
            {
                DBGERR(INEVENT,"Low Battery Event occurred!!!\r\n");
                sendMcCommand(MC_LOWBATT_OCCURR_EVT);
            }
            break;

        case INEVENT_CHARGE_STATUS_ADC_EVT:
            
            {
                if( adc_val <= CHARGEADC_LOW)  // charging....
                {
                    if(IsExtPwrInState == TRUE)
                    {
                        if(IsCharging != 1)
                        {
                            DBGHI(INEVENT,"Charging Started !!!\r\n");
                            sendMcCommand(MC_CHARGING_STARTED_EVT);
                            
                            // TODO
                        }
                        IsCharging = 1;
                    }
                }
                else if(adc_val >= CHARGEADC_HIGH) //charge complete
                {
                    if(IsExtPwrInState == TRUE)
                    {
                        if(IsCharging != 0)
                        {
                            DBGHI(INEVENT,"Charge Complete !!!\r\n");
                            sendMcCommand(MC_FULL_CHARGE_EVT);
                        }
                    }        
                    else
                    {
                        if(IsCharging == 1) 
                        {
                            DBGHI(INEVENT,"Charging Stopped !!!\r\n");
                            sendMcCommand(MC_CHARGING_STOPED_EVT);
                        }
                    }
                    IsCharging = 0;
                }
                else //  CHARGEADC_LOW < adc_val < CHARGEADC_HIGH // No USB
                {
                    if(IsCharging == 1) 
                    {
                        DBGHI(INEVENT,"Charging Stopped !!!\r\n");
                        sendMcCommand(MC_CHARGING_STOPED_EVT);
                    }
                    IsCharging = 0;
                }
            }
            break;

        default:
            break;
    }
}
/*===============================================================================================*/
