/*
 *  delay.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DELAY_H
#define DELAY_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"



#define millis(a1) 	HAL_GetTick(a1)
#define delay(a2) 	HAL_Delay(a2)


void delay_ns(uint32_t ns);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

//uint32_t millis();


#ifdef __cplusplus
}
#endif


#endif
