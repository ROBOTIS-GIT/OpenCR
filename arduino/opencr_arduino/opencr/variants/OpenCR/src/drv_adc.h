/*
 *  drv_adc.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


#include "util.h"



extern ADC_HandleTypeDef hADC3;



int drv_adc_init();



#ifdef __cplusplus
}
#endif


#endif

