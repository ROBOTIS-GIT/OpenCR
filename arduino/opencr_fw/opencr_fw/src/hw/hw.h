/*
 *  hw.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef HW_H
#define HW_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


#include "core/usb.h"
#include "core/vcp.h"
#include "core/wdg.h"
#include "core/micros.h"
#include "core/millis.h"
#include "core/delay.h"
#include "core/led.h"


void hwInit(void);



#ifdef __cplusplus
}
#endif


#endif
