/*===============================================================================================*/
/**
 *   @file mc_process.c
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
#include "mc_process.h"
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
static uint16_t pwrKeyCount=0;

/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/
/*-----------------------------------------------------------*/
void sendTaskStartSignal()
{
    command_type    cmd;

    portENTER_CRITICAL();
    cmd.len = 0;
    cmd.msg = NULL;
    cmd.isconst = FALSE;

    cmd.cmd = INEVENT_TASK_START_CMD_F;
    xQueueSend(xInEventTaskQueue,&cmd,taskNO_BLOCK);

    cmd.cmd = OUTEVENT_TASK_START_CMD_F;
    xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
    
    portEXIT_CRITICAL();
}
           

/*-----------------------------------------------------------*/

void check_pwronstate()
{
#if 1
    uint16_t    loop_count=0, auto_count=0;

    for(;;)
    {
        if( HAL_GPIO_ReadPin(KEY_PWR_GPIO_Port,KEY_PWR_Pin) == GPIO_PIN_SET)
        {
            auto_count=0;
            if(loop_count >=5)
            {
                HAL_GPIO_WritePin(PW_CTRL_GPIO_Port, PW_CTRL_Pin, GPIO_PIN_SET);
                break;
            }
            loop_count++;
        }
        else
        {
            loop_count=0;
        #ifdef USB_PWR_AUTO_ON
            if(auto_count >=2)
            {
                HAL_GPIO_WritePin(PW_CTRL_GPIO_Port, PW_CTRL_Pin, GPIO_PIN_SET);
                break;
            }
            auto_count++;            
        #endif
        }
        vTaskDelay( 100); // Delay 100msec
    } 
#else
    HAL_GPIO_WritePin(PW_CTRL_GPIO_Port, PW_CTRL_Pin, GPIO_PIN_SET);
#endif
}

/*-----------------------------------------------------------*/
boolean check_pwroffstate(void)
{

    if( HAL_GPIO_ReadPin(KEY_PWR_GPIO_Port,KEY_PWR_Pin) == GPIO_PIN_SET)
    {
        if( pwrKeyCount++ > 4) // 3sec
        {
            return TRUE;
        }

        /* Restart the xmcPwrKeyCheckTimer. */
        xTimerStart( xmcPwrKeyCheckTimer, 0 );
    }
    else
    {
        pwrKeyCount = 0;
    }

    return FALSE;
}

/*-----------------------------------------------------------*/
void sendSTOPandPowerOFF(void)
{

	command_type	cmd;

    portENTER_CRITICAL();
	cmd.len = 0;
	cmd.msg = NULL;
    cmd.isconst = FALSE;
    
	cmd.cmd = INEVENT_TASK_STOP_F;
	xQueueSend(xInEventTaskQueue,&cmd,taskNO_BLOCK);
	vTaskDelay( 5);

	cmd.cmd = OUTEVENT_TASK_STOP_F;
	xQueueSend(xOutEventTaskQueue,&cmd,taskNO_BLOCK);
	vTaskDelay( 5);

    portEXIT_CRITICAL();

	vTaskDelay( 500); // Delay 3sec
	
    HAL_GPIO_WritePin(PW_CTRL_GPIO_Port, PW_CTRL_Pin, GPIO_PIN_RESET);
    // Power OFF

}
 

/*-----------------------------------------------------------*/

/*===========================================================================
FUNCTION           
DESCRIPTION     
DEPENDENCIES
RETURN VALUE
===========================================================================*/
void PlasmaVolSend2Task(command_type *cmd)
{    
    uint8_t *data;
    
    if((data = (uint8_t *)cmd_malloc(FDPOS(outevent_cmd_msg_field_type,EndofData))) != NULL)
    {
        command_type            scmd;
        uint8_t *idata = (uint8_t *)cmd->msg;
        
        scmd.len = FDPOS(outevent_cmd_msg_field_type,EndofData);
        scmd.isconst = FALSE;
        scmd.cmd = PLASMA_VOL_CTRL_F;
    
        data[FDPOS(outevent_cmd_msg_field_type,Cmd)] = (uint8_t)idata[FDPOS(outevent_cmd_msg_field_type,Cmd)];
        scmd.msg = data;
    
        portENTER_CRITICAL();
        xQueueSend(xOutEventTaskQueue,&scmd,taskNO_BLOCK);
        portEXIT_CRITICAL();
    }
    else
    {
        DBGERR(MC,"Memory Allocation Error!!!\r\n");
    }       
}
/*===============================================================================================*/
