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




extern ADC_HandleTypeDef hADC1;
extern ADC_HandleTypeDef hADC3;



int drv_adc_init();
void drv_adc_pin_init( uint32_t ulPin );


#ifdef __cplusplus
}
#endif


#endif
