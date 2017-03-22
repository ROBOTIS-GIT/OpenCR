/*
 *  drv.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DRV_MICROS_H
#define DRV_MICROS_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"


void drv_micros_init();

uint32_t drv_micros();



#ifdef __cplusplus
}
#endif


#endif
