/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif


	/*
		There is enough abstract similarity to Arduino that for some of these
		we can just do the function with a define in the chip.h header

		#define millis HAL_GetTick
		#define delay HAL_Delay
	*/


/* 
	Not sure how to do micros when the system clock ticks in milliseconds
	
	
*/ 
uint32_t micros( void )
{
	/* dead function implementation Return maximum time as error */
	return -1;
}

// original function:
// uint32_t micros( void )
// {
//     uint32_t ticks ;
//     uint32_t count ;
// 
//     SysTick->CTRL;
//     do {
//         ticks = SysTick->VAL;
//         count = GetTickCount();
//     } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
// 
//     return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock/1000000) ;
// }



#ifdef __cplusplus
}
#endif
