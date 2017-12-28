#ifndef COMDEF_H
#define COMDEF_H
/*===============================================================================================*/
/**
 *   @file comdef.h
 *
 *   @version v1.0
 */
/*=================================================================================================

 (c) Copyright , All Rights Reserved

Revision History:

Modification Tracking
Author          Date            Number          Description of Changes
--------        --------        -------         ------------------------


===================================================================================================
 INCLUDE FILES
=================================================================================================*/



/*=================================================================================================
 CONSTANTS
=================================================================================================*/

#ifndef TRUE
#define TRUE	1   /* true value. */
#endif

#ifndef FALSE
#define FALSE	0   /* false value. */
#endif

#ifndef ON
#define ON		1    /* On value. */
#endif

#ifndef OFF
#define OFF		0    /* Off value. */
#endif

#ifndef NULL
#define NULL	0
#endif

typedef  unsigned char      boolean;        /* Boolean value type. */
typedef  unsigned int       uint32;      /* Unsigned 32 bit value */
typedef  unsigned short     uint16;      /* Unsigned 16 bit value */
typedef  unsigned char      uint8;       /* Unsigned 8  bit value */
typedef  signed int         int32;       /* Signed 32 bit value */
typedef  signed short       int16;       /* Signed 16 bit value */
typedef  signed char        int8;        /* Signed 8  bit value */
typedef  float              FP32;        /* Single precision floating point */
typedef  double             FP64;        /* Double precision floating point */


#ifndef MIN
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef FDPOS
    #define FDPOS( type, field ) ( (uint32) &(( type *) 0)-> field )
#endif

#ifndef FDSIZ
    #define FDSIZ( type, field ) sizeof( ((type *) 0)->field )
#endif    

/*===============================================================================================*/
#endif  /* COMDEF_H */
