/*===============================================================================================*/
/**
 *   @file Buzzer_state.c
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
#include "signals.h"
#include "command.h"
#include "task_cfg.h"
#include "Buzzer_state.h"
#include "debugmsgcli.h"
#include "interface.h"
#include "timergen.h"
#include "timers.h"
#include "tim.h"

/*=================================================================================================
 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
void Buzzer_Stop(void);
void Buzzer_Start(BZ_sound_type * sound);


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
static uint32           buzzer_data_index=0;
static boolean          onoffState;

static BZ_sound_type    *BZ_SOUND;

TimerHandle_t xbuzzerTimer = NULL;

// buzzer Data
const BZ_sound_data_type Def_sound0[] =
{
    {  175, 2000,    0 },   //  1/175 = 5.714 KHz, 175*2000= 350msec on, 175*0= 0 msec off
    {    0,    0,    0 }    // end of sound
};

const BZ_sound_data_type Def_sound1[] =
{
    {  175, 1000, 1000 },   //  1/175 = 5.714 KHz, 175*20000= 3500msec on, 175*0= 0 msec off
    {  175, 1000, 1000 },   //  1/175 = 5.714 KHz, 175*20000= 3500msec on, 175*0= 0 msec off
    {  175, 5000, 5000 },   //  1/175 = 5.714 KHz, 175*20000= 3500msec on, 175*0= 0 msec off
    {    0,    0,    0 }    // end of sound
};

const BZ_sound_data_type Def_sound2[] =
{
    {  1911, 157,    0 },   //
    {  1804, 166,    0 },   
    {  1703, 176,    0 },   
    {  1607, 187,    0 },  
    {  1517, 198,    0 },   
    {  1423, 209,    0 },   
    {  1351, 222,    0 },   
    {  1276, 235,    0 },   
    {     0,   0,    0 }    // end of sound
};

const BZ_sound_data_type Def_sound3[] =
{
    {  956, 314,    0 }, //
    {  902, 333,    0 }, // 1046.502 *2^(1/12) = 902   
    {  851, 353,    0 },   
    {  804, 373,    0 },   
    {  758, 396,    0 },   
    {  716, 419,    0 },   
    {  676, 444,    0 },   
    {  638, 470,    0 },    
    {    0,   0,    0 }    // end of sound
};

const BZ_sound_data_type Def_sound4[] =
{
    {  478, 628,    0 }, // 7
    {  451, 665,    0 },   
    {  426, 704,    0 },   
    {  402, 746,    0 },   
    {  379, 792,    0 },   
    {  358, 838,    0 },   
    {  338, 888,    0 },   
    {  319, 940,    0 },   
    {    0,   0,    0 }    // end of sound
};

const BZ_sound_data_type Def_sound5[] =
{
    {  239, 1255,    0 }, // 8
    {  225, 1333,    0 },   
    {  213, 1408,    0 },   
    {  201, 1493,    0 },   
    {  190, 1579,    0 },   
    {  179, 1676,    0 },   
    {  169, 1775,    0 },   
    {  159, 1887,    0 },     
    {    0,    0,    0 }    // end of sound
};

BZ_sound_type Default_bz_sound0 = 
{ 
    FALSE,
    Def_sound0
};

BZ_sound_type Default_bz_sound1 = 
{ 
    FALSE,
    Def_sound1
};

BZ_sound_type Default_bz_sound1_1 = 
{ 
    FALSE,
    Def_sound1
};

BZ_sound_type Default_bz_sound2 = 
{ 
    FALSE,
    Def_sound2
};

BZ_sound_type Default_bz_sound3 = 
{ 
    FALSE,
    Def_sound3
};

BZ_sound_type Default_bz_sound4 = 
{ 
    FALSE,
    Def_sound4
};

BZ_sound_type Default_bz_sound5 = 
{ 
    FALSE,
    Def_sound5
};

/*==================================================================================================
 GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
 LOCAL FUNCTIONS
==================================================================================================*/
void Buzzer_Start(BZ_sound_type * sound)
{
    Buzzer_Stop();
    BZ_SOUND = sound;
    
    buzzer_data_index = 0;
    onoffState = TRUE;

    timer_set_timer(xbuzzerTimer,BZ_SOUND->sound[0].ONPulseTime);
    MX_TIM3_Init(BZ_SOUND->sound[0].OnePerFreq);

    /* TIM3 enable counter */
    //HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}

void Buzzer_Start_def()
{
    Buzzer_Start(&Default_bz_sound1);
}

void Buzzer_Stop(void)
{
    timer_clr_timer(xbuzzerTimer);
    /* TIM3 disable counter */
    //HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
    //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void Buzzer_check_buzzer(void)
{
    boolean isNextLoad = FALSE;

    if( (BZ_SOUND->sound[buzzer_data_index].OnePerFreq == 0) && 
        (BZ_SOUND->sound[buzzer_data_index].ONPulseTime == 0) && 
        (BZ_SOUND->sound[buzzer_data_index].OFFPulseTime == 0)
      )
    {
        if(BZ_SOUND->repeat == TRUE)
        {
            buzzer_data_index = 0;
            onoffState = TRUE;
            isNextLoad = TRUE;
        }
        else
        {
            // stop sound
            Buzzer_Stop();
            return;
        }
    }
        
    if(onoffState == TRUE)
    {
        if(BZ_SOUND->sound[buzzer_data_index].OFFPulseTime != 0)
        {
            onoffState = FALSE;
            timer_set_timer(xbuzzerTimer,BZ_SOUND->sound[buzzer_data_index].OFFPulseTime);     
            
            HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
            //HAL_TIM_Base_Stop(&htim3);
        }
        else
        {
            isNextLoad = TRUE;
            buzzer_data_index++;
        }
    }
    else
    {
        isNextLoad = TRUE;
        buzzer_data_index++;
    }

    if(isNextLoad == TRUE)
    {
        onoffState = TRUE;
        MX_TIM3_Init(BZ_SOUND->sound[buzzer_data_index].OnePerFreq);
        
        //HAL_TIM_Base_Start(&htim3);
        HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

        if(BZ_SOUND->sound[buzzer_data_index].ONPulseTime != 0)
        {
            timer_set_timer(xbuzzerTimer,BZ_SOUND->sound[buzzer_data_index].ONPulseTime);
        }
        else
        {
            DBGERR(OUTEVENT,"Buzzer Sound Error!!! index()\r\n",buzzer_data_index, BZ_SOUND->sound[buzzer_data_index].ONPulseTime);
        }
    }
}

void Buzzer_select(uint8_t type)
{
    switch(type)
    {
        case 1:
            Buzzer_Start(&Default_bz_sound2);
            break;

        case 2:
            Buzzer_Start(&Default_bz_sound3);
            break;            
            
        case 3:
            Buzzer_Start(&Default_bz_sound4);
            break;            

        default:
            Buzzer_Start(&Default_bz_sound0);
            break;
    }
}


/*==================================================================================================
 GLOBAL FUNCTIONS
==================================================================================================*/

void BuzzerState_init(TimerCallbackFunction_t pxCallbackFunction)
{
	/* Create a one-shot timer for use later on in this test. */
	xbuzzerTimer = xTimerCreate(	"",				/* Text name to facilitate debugging.  The kernel does not use this itself. */
								250,					/* The period for the timer(1 sec). */
								pdFALSE,				/* Don't auto-reload - hence a one shot timer. */
								( void * )BUZZER_TIMER_1_F,	/* The timer identifier. */
								pxCallbackFunction );	/* The callback to be called when the timer expires. */

    // Start Buzzer module trigger
    //sendtoOutEventCmd(BUZZER_EVENT_START_F);
}
    
boolean CheckIsBuzzerCmd(command_type *command)
{
    boolean         isprocessed = FALSE;
    uint8_t         *data = (uint8_t *)command->msg;
    
    if(ISBUZZERSTATE_EVENT(command->cmd))
    {
        isprocessed = TRUE;
        
        switch(command->cmd)
        {
            case BUZZER_TIMER_1_F:
                Buzzer_check_buzzer();
                break;
                
        	case BUZZER_ON_F:
                Buzzer_Start_def();
                DBGHI(OUTEVENT,"Default buzzer on.\r\n");
				break;
				
            case BUZZER_SOUND_OFF_F:
			case BUZZER_OFF_F:
                timer_clr_timer(xbuzzerTimer);
                Buzzer_Stop();
                DBGHI(OUTEVENT,"Buzzer Stopped.\r\n");
				break;
						
			case BUZZER_SOUND_ON_F:
                Buzzer_select( (uint8_t)data[FDPOS(gen_cmd_msg_field_type,data1)]);
                DBGHI(OUTEVENT,"Buzzer Sound %d Started.\r\n",(uint8_t)data[FDPOS(gen_cmd_msg_field_type,data1)]);
				break;
				
			default:
				break;
        }
    }

    return isprocessed;
}    

/*===============================================================================================*/
