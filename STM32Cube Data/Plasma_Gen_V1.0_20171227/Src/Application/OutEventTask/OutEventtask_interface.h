#ifndef OUTEVENTTASK_INTERFACE_H
#define OUTEVENTTASK_INTERFACE_H
/*===============================================================================================
 *
 *   @file OutEventtask_interface.h
 *
 *   @version v1.0
 *
 =================================================================================================


Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------

Portability:
Indicate if this module is portable to other compilers or
platforms. If not, indicate specific reasons why is it not portable.
*/

/*================================================================================================
 INCLUDE FILES
=================================================================================================*/

/*=================================================================================================
 CONSTANTS
=================================================================================================*/


/*=================================================================================================
 ENUMS
=================================================================================================*/
typedef enum {
	POWER_ON_SOUND,
	POWER_OFF_SOUND,
	LOW_BATT_SOUND,
	EXT_PWR_IN_SOUND,
	EXT_PWR_OUT_SOUND,
	SIMPLE_SOUND1,
	SIMPLE_SOUND2,
	SOUND_SOUND_MAX=0xFF
}outevent_sound_type;

/*=================================================================================================
 STRUCTURES AND OTHER TYPEDEFS
=================================================================================================*/

typedef struct {
    uint8_t             Cmd[1];
    uint8_t             Dummy[1];
    uint8_t             EndofData[1];
} outevent_cmd_msg_field_type;


/*===============================================================================================*/
#endif  /* OUTEVENTTASK_INTERFACE_H*/
