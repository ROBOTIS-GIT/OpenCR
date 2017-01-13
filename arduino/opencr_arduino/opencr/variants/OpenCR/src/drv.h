/*
 *  drv.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DRV_H
#define DRV_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "drv_adc.h"
#include "drv_pwm.h"
#include "drv_spi.h"
#include "drv_uart.h"
#include "drv_i2c.h"
#include "drv_exti.h"
#include "drv_dxl.h"
#include "drv_timer.h"
#include "drv_eeprom.h"
#include "drv_rtc.h"


int drv_init(void);

uint32_t drv_micros();


#ifdef __cplusplus
}
#endif


#endif
