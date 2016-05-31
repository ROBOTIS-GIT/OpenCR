/*
 *  util.c
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "util.h"





uint32_t millis()
{
  return HAL_GetTick();
}
