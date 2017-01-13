/*
 *  drv_rtc.h
 *
 *  Created on: 2017. 1.13.
 *      Author: Baram
 */

#ifndef DRV_RTC_H
#define DRV_RTC_H


#ifdef __cplusplus
 extern "C" {
#endif

#include <time.h>

#include "def.h"
#include "bsp.h"


#include "util.h"



// interface for arduino
time_t rtcGetTime();
void   rtcSetTime(time_t time_data);


int drv_rtc_init();


#ifdef __cplusplus
}
#endif


#endif
