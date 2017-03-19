/*
 *  bsp.h
 *
 *  boart support package
 *
 *  Created on: 2017. 3. 16.
 *      Author: Baram
 */

#ifndef BSP_H
#define BSP_H


#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>

#include "def.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "system_clock.h"


#define USE_USB_FS





void bspInit();
void bspDeinit();



#ifdef __cplusplus
}
#endif
#endif
