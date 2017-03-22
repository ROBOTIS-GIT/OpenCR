/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#include "WInterrupts.h"
#include "variant.h"
#include "wiring_digital.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif



/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 */

void attachInterrupt( uint32_t pin, voidFuncPtr callback, uint32_t ulMode )
{
  drv_exti_attach( pin, callback, ulMode );
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt( uint32_t pin )
{
  drv_exti_detach( pin );
}


#ifdef __cplusplus
}
#endif
