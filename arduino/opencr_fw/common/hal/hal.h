/*
 *  hal.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef HAL_H
#define HAL_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


#include "util.h"
#include "delay.h"
#include "flash.h"
#include "vcp.h"
#include "msg.h"



void hal_init();



#ifdef __cplusplus
}
#endif


#endif

