/*
 *  def.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DEF_H
#define DEF_H

#include <stdint.h>
#include <stdbool.h>

#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*
#ifndef bool
#define bool uint8_t
#endif

#ifndef true
#define true  1
#endif

#ifndef false
#define false 0
#endif

*/

typedef void (*voidFuncPtr)(void);


#include "def_err.h"




#define _DEF_CAN1               0
#define _DEF_CAN2               1
#define _DEF_CAN_BAUD_125K      0
#define _DEF_CAN_BAUD_250K      1
#define _DEF_CAN_BAUD_500K      2
#define _DEF_CAN_BAUD_1000K     3
#define _DEF_CAN_STD            0
#define _DEF_CAN_EXT            1





#endif
