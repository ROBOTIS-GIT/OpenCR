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





// interface for arduino
time_t rtcGetTime();
void   rtcSetTime(time_t time_data);


int drv_rtc_init();
void     drv_rtc_write_step(uint32_t step_data);
uint32_t drv_rtc_read_step(void);

#ifdef __cplusplus
}
#endif


#endif
