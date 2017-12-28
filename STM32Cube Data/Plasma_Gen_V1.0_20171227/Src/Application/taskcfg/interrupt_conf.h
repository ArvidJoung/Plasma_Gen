#ifndef INTERRUPT_CONF_H
#define INTERRUPT_CONF_H
/*===============================================================================================*/
/**
 *   @file interrupt_conf.h
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

=================================================================================================
 INCLUDE FILES
=================================================================================================*/

/* Kernel includes. */
#include "FreeRTOS.h"

/*=================================================================================================
 CRITICAL CONSTANTS
=================================================================================================*/
/*
    Refer to FreeRTOSConfig.h

	The lowest interrupt priority that can be used in a call to a "set priority"
	function. 
	#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

	The highest interrupt priority that can be used by any interrupt service
	routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
	INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
	PRIORITY THAN THIS! (higher priorities are lower numeric values. 
	#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

    RTOS_API_INTERRUPT_GROUPx must be set numeric high(Logical low) value than  configMAX_SYSCALL_INTERRUPT_PRIORITY(priority 11)

    #define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
    #define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    11

    So interrupts that use ISR safe FreeRTOS calls can use priorities 11, 12, 13 14 and 15. Priorities above 11 never get disabled.

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;¡±


    !!!!!!!!
    See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
    !!!!!!!!
*/

// CATION !!!
/*!< STM32F0xx uses 2 Bits for the Priority Levels */

#if(0)
#define RTOS_API_INTERRUPT_GROUP1               (configLIBRARY_LOWEST_INTERRUPT_PRIORITY-1)
#define RTOS_API_INTERRUPT_GROUP2               (configLIBRARY_LOWEST_INTERRUPT_PRIORITY-2)
#define RTOS_API_INTERRUPT_GROUP3               (configLIBRARY_LOWEST_INTERRUPT_PRIORITY-3)
#else
#define RTOS_API_INTERRUPT_GROUP                (configLIBRARY_LOWEST_INTERRUPT_PRIORITY)
#endif

#if(0)
#define NO_RTOS_API_INTERRUPT_GROUP1            9
#define NO_RTOS_API_INTERRUPT_GROUP2            8
#define NO_RTOS_API_INTERRUPT_GROUP3            7
#define NO_RTOS_API_INTERRUPT_GROUP4            6
#else
#define NO_RTOS_API_INTERRUPT_GROUP1            (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-1)
#endif


/*===============================================================================================*/
#endif  /* INTERRUPT_CONF_H */
