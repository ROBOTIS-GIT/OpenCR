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





void bsp_init();
void bsp_deinit();

void bsp_mpu_config(void);

#ifdef __cplusplus
}
#endif
#endif
