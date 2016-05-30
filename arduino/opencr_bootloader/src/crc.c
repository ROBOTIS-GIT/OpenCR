/*
 *  crc.c
 *
 *  crc
 *
 *  Created on: 2016. 5. 30.
 *      Author: Baram, PBHP
 */

#include "crc.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>




/*---------------------------------------------------------------------------
     TITLE   : crc_calc
     WORK    :
---------------------------------------------------------------------------*/
uint32_t crc_calc( uint32_t crc_in, uint8_t data_in )
{

  crc_in  ^= data_in;
  crc_in  += data_in;

  return crc_in;
}
