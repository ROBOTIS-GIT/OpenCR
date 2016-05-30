/*
 *  crc.h
 *
 *  command process
 *
 *  Created on: 2016. 5. 30.
 *      Author: Baram, PBHP
 */

#ifndef CRC_H
#define CRC_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"
#include "hal.h"





uint32_t crc_calc( uint32_t crc_in, uint8_t data_in );



#ifdef __cplusplus
}
#endif


#endif

