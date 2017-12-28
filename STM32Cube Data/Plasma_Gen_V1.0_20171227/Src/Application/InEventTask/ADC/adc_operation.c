/*===============================================================================================*/
/**
 *   @file adc_operation.c
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

===================================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Standard includes. */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f0xx_hal.h"

/* Application includes. */
#include "target.h"
#include "signals.h"
#include "command.h"
#include "adc.h"
#include "adc_operation.h"
#include "comdef.h"
#include "stdlib.h"
#include "debugmsgcli.h"
#include "gpio.h"


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
uint8_t     BatteryStatus;

/* PC0     CHAN9  Charging Status*/
uint8_t     ChargingStatus;

/* PC2     CHAN12  Battery Temp*/
uint8_t     BatteryTemp;

/* PC3     CHAN13  PLA Control*/
uint8_t     Plasma_vol;


/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/
uint32_t Get_ADC_HWChannel(uint8_t chan)
{
    uint32_t     ret;
    
    switch(chan)
    {
        case ADC_PLA_VOL:
            ret = ADC_CHANNEL_13;
            break;
            
        case ADC_BATT:
            ret = ADC_CHANNEL_11;
            break;
            
        case ADC_CHARGE:
            ret = ADC_CHANNEL_9;
            break;

        case ADC_BATT_TEMP:
            ret = ADC_CHANNEL_12;
            break;
            
        default:
            ret = 0xFF;
            break;
    }

    return ret;
}


void Set_ADCRankNone(uint8_t chan)
{
    ADC_ChannelConfTypeDef sConfig;

    if(( sConfig.Channel = Get_ADC_HWChannel(chan)) != 0xFF)
    {
        sConfig.Rank = ADC_RANK_NONE;
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
    }
}


boolean ReadAdcResult(uint16_t *data, uint8_t chan)
{   
    #if defined (ADC_USE_POLLING_METHOD)
    /*##-4- Wait for the end of conversion #####################################*/
    if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
    {
        Set_ADCRankNone(chan);
      /* End Of Conversion flag not set on time */
      return FALSE;
    }
    #endif
    
    /* Check if the continuous conversion of regular channel is finished */
    if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
      /*##-5- Get the converted value of regular channel  ########################*/
      *data =  HAL_ADC_GetValue(&hadc);
      Set_ADCRankNone(chan);      
      return TRUE;
    }
    Set_ADCRankNone(chan);    
    return FALSE;   
}


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/    
boolean adcStartAdc(uint8_t chan)
{
    ADC_ChannelConfTypeDef sConfig;
    
    /*##-2- Configure ADC regular channel ######################################*/
    if(( sConfig.Channel = Get_ADC_HWChannel(chan)) != 0xFF)
    {
        sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;    /* Sampling time (number of clock cycles unit) */

        if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
        {
          /* Channel Configuration Error */
          //Error_Handler();
        }

        /*##-3- Start the conversion process #######################################*/
        
#if defined (ADC_USE_POLLING_METHOD)
        HAL_ADC_Start(&hadc);
#elif defined (ADC_USE_IT_METHOD)
        HAL_ADC_Start_IT(&hadc);
#endif

#if(0)
     /*##-4- Wait for the end of conversion #####################################*/
      /*  Before starting a new conversion, you need to check the current state of
           the peripheral; if it’s busy you need to wait for the end of current
           conversion before starting a new one.
           For simplicity reasons, this example is just waiting till the end of the
           conversion, but application may perform other tasks while conversion
           operation is ongoing. */
      if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
      {
        /* End Of Conversion flag not set on time */
        Error_Handler();
      }

    /* Check if the continuous conversion of regular channel is finished */
      if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
      {
        /*##-5- Get the converted value of regular channel  ########################*/
        uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
      }
#endif
        return TRUE;
    }
    else
    {
        return FALSE;
    }   
}


/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/   
boolean Read_ADC_value(uint8_t chan)
{
    boolean     ret = TRUE;
    uint16_t    adc_val;
    
        
    switch(chan)
    {
        case ADC_PLA_VOL:
            if( ReadAdcResult(&adc_val,chan) == TRUE)
            {
                Plasma_vol = (adc_val >> 4) & 0xFF;
                DBGMED(INEVENT,"Batt ADC %d BatteryStatus=%d\r\n",adc_val,Plasma_vol);
            }
            break;
            
        case ADC_BATT:
            if( ReadAdcResult(&adc_val,chan) == TRUE)
            {
                BatteryStatus = (adc_val >> 4) & 0xFF;
                DBGMED(INEVENT,"Batt ADC %d BatteryStatus=%d\r\n",adc_val,BatteryStatus);
                DBGTEST(1,"BATT ADC:%d\r\n",BatteryStatus);
            }
            break;
            
        case ADC_CHARGE:
            if( ReadAdcResult(&adc_val,chan) == TRUE)
            {
                ChargingStatus = (adc_val >> 4) & 0xFF;
                DBGMED(INEVENT,"Charge ADC %d ChargingStatus=%d\r\n",adc_val,ChargingStatus);
            }
            break;

        case ADC_BATT_TEMP:
            if( ReadAdcResult(&adc_val,chan) == TRUE)
            {
                BatteryTemp = (adc_val >> 4) & 0xFF;                
                DBGMED(INEVENT,"Batt Temp ADC %d BatteryTemp=%d\r\n",adc_val,BatteryTemp);
            }
            break;

        default:
            ret = FALSE;
            break;
    }

    return ret;
}

/*===============================================================================================*/
